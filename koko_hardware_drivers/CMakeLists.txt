cmake_minimum_required(VERSION 2.8.3)
project(koko_hardware_drivers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES koko_hardware_drivers
  CATKIN_DEPENDS roscpp rospy sensor_msgs serial std_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(BLDCControllerClient src/BLDCControllerClient.cpp)
target_link_libraries(BLDCControllerClient ${catkin_LIBRARIES})

add_executable(BLDCDriver_node src/BLDCDriver_node.cpp)
target_link_libraries(BLDCDriver_node
  ${catkin_LIBRARIES}
  BLDCControllerClient
)

add_executable(BLDCControllerTest src/BLDCControllerTest.cpp)
target_link_libraries(BLDCControllerTest
  ${catkin_LIBRARIES}
  BLDCControllerClient
)

add_executable(BLDCDriver_communication_freq_test src/BLDCDriver_communication_freq_test.cpp)
target_link_libraries(BLDCDriver_communication_freq_test
  ${catkin_LIBRARIES}
  BLDCControllerClient
)

## Mark executables and/or libraries for installation
install(TARGETS BLDCControllerClient BLDCDriver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/koko_hardware_drivers
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
