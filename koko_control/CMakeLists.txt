cmake_minimum_required(VERSION 2.8.3)
project(koko_control)

find_package(catkin REQUIRED COMPONENTS
  koko_hardware_drivers
  controller_interface
  controller_manager
  joint_limits_interface
  hardware_interface
  transmission_interface
  geometry_msgs
  kdl_parser
  roscpp
  sensor_msgs
  std_msgs
  tf
  urdf
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES koko_control
  CATKIN_DEPENDS controller_interface controller_manager geometry_msgs geometry_msgs koko_hardware_drivers kdl_parser orocos_kdl roscpp sensor_msgs std_msgs tf urdf visualization_msgs joint_limits_interface
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(koko_controller_manager src/koko_controller_manager.cpp)
target_link_libraries(koko_controller_manager ${catkin_LIBRARIES})
