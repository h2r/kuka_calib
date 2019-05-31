cmake_minimum_required(VERSION 2.8.3)
project(kuka_calib)
add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

catkin_package(DEPENDS Eigen3)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(extrinsic_calibration_node src/extrinsic_calibration.cpp)

target_link_libraries(extrinsic_calibration_node ${catkin_LIBRARIES})
