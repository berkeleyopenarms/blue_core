cmake_minimum_required(VERSION 2.8.3)
project(koko_hardware_drivers)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  serial
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES koko_hardware_drivers
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs serial
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/BLDCControllerClient.cpp
  src/BLDCDriver.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
