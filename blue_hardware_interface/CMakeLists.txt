cmake_minimum_required(VERSION 2.8.3)
project(blue_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  blue_hardware_drivers
  blue_msgs
  hardware_interface
  transmission_interface
  kdl_parser
  roscpp
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES blue_hardware_interface
  CATKIN_DEPENDS blue_hardware_drivers kdl_parser roscpp urdf blue_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(blue_hardware_interface
  src/blue_hardware_interface.cpp
  src/blue_transmissions.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blue_hardware_interface ${catkin_LIBRARIES})
### Install

## Mark executables and/or libraries for installation
install(TARGETS blue_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
