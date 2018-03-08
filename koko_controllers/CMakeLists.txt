cmake_minimum_required(VERSION 2.8.3)
project(koko_controllers)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  urdf
  angles
  kdl_parser
  sensor_msgs
  controller_manager
  geometry_msgs
  visualization_msgs
)

find_package(
  orocos_kdl
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES koko_controllers
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/InverseDynamicsController.cpp src/CartesianPoseController.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES koko_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
