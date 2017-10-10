cmake_minimum_required(VERSION 2.8.3)
project(koko_kinematics)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  kdl_parser
  interactive_markers tf visualization_msgs
)
find_package(orocos_kdl)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

add_executable(pose_target src/pose_target.cpp)
target_link_libraries(pose_target ${catkin_LIBRARIES})
