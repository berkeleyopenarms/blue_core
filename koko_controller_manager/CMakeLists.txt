cmake_minimum_required(VERSION 2.8.3)
project(koko_controller_manager)

find_package(catkin REQUIRED COMPONENTS
  koko_hardware_drivers
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
  LIBRARIES koko_control
  CATKIN_DEPENDS controller_interface controller_manager koko_hardware_drivers kdl_parser roscpp urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(koko_hardware_interface
  src/koko_hardware_interface.cpp
)
target_link_libraries(koko_hardware_interface ${catkin_LIBRARIES})

add_executable(koko_controller_manager src/koko_controller_manager.cpp)
target_link_libraries(koko_controller_manager koko_hardware_interface ${catkin_LIBRARIES})

### Install

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS koko_controller_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
