cmake_minimum_required(VERSION 2.8.3)
project(blue_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(FILES MotorState.msg GravityVectorArray.msg)
add_service_files(FILES JointStartupCalibration.srv GripperPositionCalibration.srv InverseKinematics.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()
