cmake_minimum_required(VERSION 2.8.3)
project(blue_controller_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  blue_hardware_drivers
  blue_msgs
  controller_interface
  controller_manager
  hardware_interface
  transmission_interface
  kdl_parser
  roscpp
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller_interface controller_manager blue_hardware_drivers kdl_parser roscpp urdf blue_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(blue_hardware_interface
  src/blue_hardware_interface.cpp
)
target_link_libraries(blue_hardware_interface ${catkin_LIBRARIES})

add_executable(blue_controller_manager src/blue_controller_manager.cpp)
target_link_libraries(blue_controller_manager blue_hardware_interface ${catkin_LIBRARIES})
### Install

install(PROGRAMS scripts/spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS blue_controller_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
