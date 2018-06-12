cmake_minimum_required(VERSION 2.8.3)
project(blue_hardware_drivers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(FILES MotorState.msg GravityVectArr.msg)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES blue_hardware_drivers
  CATKIN_DEPENDS roscpp rospy sensor_msgs serial std_msgs message_runtime
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/BLDCControllerClient.cpp
  src/BLDCDriver.cpp
  src/Packets.cpp
  src/crc16.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(BLDCDriver_node src/BLDCDriver_node.cpp)
#target_link_libraries(BLDCDriver_node
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

add_executable(BLDCControllerTest src/BLDCControllerTest.cpp)
target_link_libraries(BLDCControllerTest
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(BLDCDriver_communication_freq_test src/BLDCDriver_communication_freq_test.cpp)
target_link_libraries(BLDCDriver_communication_freq_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
#install(TARGETS BLDCDriver_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS BLDCDriver_communication_freq_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/blue_hardware_drivers
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark Python scripts for installation
install(PROGRAMS scripts/comms.py scripts/bldc_driver.py scripts/bldc_driver_frequency_test.py scripts/accel_gravity_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
