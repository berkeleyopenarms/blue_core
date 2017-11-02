cmake_minimum_required(VERSION 2.8.3)
project(koko_control)

find_package(catkin REQUIRED COMPONENTS
  # angle
  # controller
  controller_manager
  geometry_msgs
  geometry_msgs
  koko_hardware_drivers
  # interface
  kdl_parser
  orocos_kdl
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
#  CATKIN_DEPENDS angle controller controller_manager geometry_msgs geometry_msgs interface kdl_parser orocos_kdl roscpp sensor_msgs std_msgs tf urdf visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(koko_controller_manager src/koko_controller_manager.cpp)
target_link_libraries(koko_controller_manager ${catkin_LIBRARIES})

add_executable(koko_controller_manager_independent src/koko_controller_manager_independent.cpp)
target_link_libraries(koko_controller_manager_independent ${catkin_LIBRARIES})