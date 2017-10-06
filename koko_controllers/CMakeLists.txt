cmake_minimum_required(VERSION 2.8.3)
project(koko_controllers)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  urdf 
  angles
  kdl_parser
  sensor_msgs
  controller_manager
  geometry_msgs
  visualization_msgs
)

find_package(
  orocos_kdl
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES koko_controllers
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(InverseDynamicsController src/InverseDynamicsController.cpp)
target_link_libraries(InverseDynamicsController ${catkin_LIBRARIES})