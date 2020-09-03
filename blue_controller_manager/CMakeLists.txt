cmake_minimum_required(VERSION 3.0.2)
project(blue_controller_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  blue_hardware_interface
  blue_msgs
  controller_interface
  controller_manager
  roscpp
)

catkin_package(
  CATKIN_DEPENDS controller_interface controller_manager blue_hardware_interface roscpp blue_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# target_link_libraries(blue_hardware_interface ${catkin_LIBRARIES})

add_executable(blue_controller_manager src/blue_controller_manager.cpp)
target_link_libraries(blue_controller_manager ${catkin_LIBRARIES})
### Install

install(PROGRAMS scripts/spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS blue_controller_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
