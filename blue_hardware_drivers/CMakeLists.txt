cmake_minimum_required(VERSION 3.0.2)
project(blue_hardware_drivers)

# add modules for finding JSONCPP
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

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
  blue_msgs
)
find_package(JSONCPP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES blue_hardware_drivers
  CATKIN_DEPENDS roscpp rospy sensor_msgs serial std_msgs
  DEPENDS JSONCPP
)
include_directories(include include/blue_hardware_drivers)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/BLDCControllerClient.cpp
  src/BLDCDriver.cpp
  src/Packets.cpp
  src/crc16.cpp
  src/Buffer.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${JSONCPP_LIBRARY}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(BLDCDriver src/BLDCDriver.cpp)
#target_link_libraries(BLDCDriver
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

#add_executable(BLDCControllerTest src/BLDCControllerTest.cpp)
#target_link_libraries(BLDCControllerTest
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

add_executable(BLDCDriver_communication_freq_test src/BLDCDriver_communication_freq_test.cpp)
target_link_libraries(BLDCDriver_communication_freq_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS BLDCDriver_communication_freq_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark Python scripts for installation
install(PROGRAMS scripts/comms.py scripts/bldc_driver.py scripts/bldc_driver_frequency_test.py scripts/accel_gravity_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
