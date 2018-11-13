cmake_minimum_required(VERSION 2.8.3)
project(blue_controllers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  urdf
  kdl_parser
  control_toolbox
  realtime_tools
  control_msgs
)

find_package(
  orocos_kdl
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS controller_interface kdl_parser roscpp urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/BlueJointGroupPositionController.cpp src/CartesianPoseController.cpp src/BlueJointGroupCTC.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES blue_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
