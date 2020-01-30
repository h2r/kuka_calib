import numpy as np
import yaml
import sys
import tf.transformations as tft
import argparse

def opticalTF2LinkTF(optical_TF):
    # Input
    # optical_TF: transform generated by calibration. Moves points from camera n optical frame to
    #    camera n+1 optical frame
    # Returns
    # link_TF: transform that moves points from camera n+1 link frame to camera n link frame

    # Realsense D435's cam_#_link to cam_#_color_optical_frame, from tf_echo
    link_to_optical_position = [-0.001, 0.015, 0.000]
    # qx qy qz qw
    link_to_optical_Quat = [-0.5, 0.5, -0.5, 0.5]

    link_to_optical =  tft.quaternion_matrix(link_to_optical_Quat)
    link_to_optical[:3,-1] = link_to_optical_position

    link_TF = np.matmul(np.matmul(link_to_optical, np.linalg.inv(optical_TF)),
                                  np.linalg.inv(link_to_optical))

    return link_TF

def decomposeTF(transform):
    # Input
    # transform: homogeneous transform between two frames
    # Returns
    # desired_trans: x,y,z distance between frames
    # desired_quat: x,y,z,w quaternion rotation between two frames

    desired_trans = transform[:3,-1]
    desired_quat = tft.quaternion_from_matrix(transform)

    return desired_trans, desired_quat

def writeTFToFile(name, T_from_base_cam, calib_dir):
    T_pos, T_rot = decomposeTF(T_from_base_cam)

    calib_filepath = calib_dir + '/' + name + "_calib.sh"

    with open(calib_filepath, 'w') as f:
        lines_to_write = ["# This file was generated by kuka_calib\n",
                          "# Transform from the base camera frame to the camera link\n",
                          "# Position\n",
                          "export " + name + "_x_pos={:.6f}\n".format(T_pos[0]),
                          "export " + name + "_y_pos={:.6f}\n".format(T_pos[1]),
                          "export " + name + "_z_pos={:.6f}\n".format(T_pos[2]),
                          "#Rotation\n",
                          "export " + name + "_x_rot={:.6f}\n".format(T_rot[0]),
                          "export " + name + "_y_rot={:.6f}\n".format(T_rot[1]),
                          "export " + name + "_z_rot={:.6f}\n".format(T_rot[2]),
                          "export " + name + "_w_rot={:.6f}\n".format(T_rot[3])]
        f.writelines(lines_to_write)

def saveTFsFromCalibFile(filename, base_camera_name, output_calib_directory):
    with open(filename) as f:
        calib_yaml_dict = yaml.load(f)

    # assume cam0 is the world frame and doesn't have a T_cn_cnm1 field
    for cam in calib_yaml_dict:
        split_rostopic = calib_yaml_dict[cam]['rostopic'].split('/')
        # first substring after possible leading slash is the camera name
        cam_name = split_rostopic[1] if split_rostopic[0] == '' else split_rostopic[0]
        calib_yaml_dict[cam]['name'] = cam_name

    # initialize link transform for first camera in chain
    calib_yaml_dict['cam0']['T_from_first_cam'] = np.eye(4)

    # transform to get points in cam_1_link frame (AKA transform from cam_1_link to cam_n_link)
    # use this transform as we iterate through to compute transforms from cam_1_link to cam_n_link
    running_transform = np.eye(4)
    # get optical frame transform that connects each camera to the chain
    for cam_num in range(len(calib_yaml_dict))[1:]:
        # each transform
        calib_dict_key = 'cam' + str(cam_num)
        
        T_cn_cnm_optical = np.array(calib_yaml_dict[calib_dict_key]["T_cn_cnm1"])
        # convert each frame from optical to link frame
        T_cn_cnm_link = opticalTF2LinkTF(T_cn_cnm_optical)
        cam_1_link_to_cam_n = np.matmul(running_transform, T_cn_cnm_link)
        running_transform = cam_1_link_to_cam_n
        calib_yaml_dict[calib_dict_key]['T_from_first_cam'] = cam_1_link_to_cam_n

    # transform from base_camera_name to first camera in chain
    base_camera_link_to_cam_1_link = None
    for cam_num in range(len(calib_yaml_dict)):
        calib_dict_key = 'cam' + str(cam_num)
        if calib_yaml_dict[calib_dict_key]['name'] == base_camera_name:
            base_camera_link_to_cam_1_link = np.linalg.inv(calib_yaml_dict[calib_dict_key]['T_from_first_cam'])
            break
    if base_camera_link_to_cam_1_link is None:
        print("Could not find specified base_camera_name:", base_camera_name)
        sys.exit()

    # now that we've computed transform from frame 0 to each other camera frame, get transform from base_camera_name
    #       to each camera
    for cam_num in range(len(calib_yaml_dict)):
        calib_dict_key = 'cam' + str(cam_num)
        base_cam_link_to_cam_n_link = np.matmul(base_camera_link_to_cam_1_link, 
                                                calib_yaml_dict[calib_dict_key]['T_from_first_cam'])
        calib_yaml_dict[calib_dict_key]['T_from_base_cam'] = base_cam_link_to_cam_n_link
    
    # write to corresponding transform bash file
    for cam_num in range(len(calib_yaml_dict)):
        calib_dict_key = 'cam' + str(cam_num)
        writeTFToFile(calib_yaml_dict[calib_dict_key]['name'],
                      calib_yaml_dict[calib_dict_key]['T_from_base_cam'],
                      output_calib_directory)

def main():
    parser = argparse.ArgumentParser(description="Calculate the transforms from a specified base camera to each " + \
        "of the other cameras in the Kuka's camera setup using calibration transforms read from kalibr's output")
    parser.add_argument('--yaml_file', type=str, default="/home/mcorsaro/camchain-2019-11-15-15-45-02_2_cam.yaml",
        help="Path to the file output by kalibr_calibrate_cameras")
    parser.add_argument('--base_camera_name', type=str, default="cam_3",
        help="Name of camera that all transforms will be generated from. cam_#")
    parser.add_argument('--output_calib_directory', type=str,
        default="/home/mcorsaro/kuka_ws/src/kuka_brown/kuka_cam/calib/",
        help="Path in which to save calibration .sh files.")
    args = parser.parse_args()

    user_response = raw_input("WARNING: RUNNING THIS SCRIPT MAY OVERWRITE ANY cam_#_calib.sh CALIBRATION FILES IN" + \
        args.output_calib_directory + ". Continue?\n")
    if user_response.lower() == "y" or user_response.lower() == "yes":
        saveTFsFromCalibFile(args.yaml_file, args.base_camera_name, args.output_calib_directory)
    else:
        print "Exiting."

if __name__ == '__main__':
    main()