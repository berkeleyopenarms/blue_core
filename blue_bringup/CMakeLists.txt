cmake_minimum_required(VERSION 2.8.3)
project(blue_bringup)

# TODO: These dependencies are nearly all for hardstop_calibrate.cpp, which will soon be removed
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  interactive_markers
  kdl_parser
  tf
  tf2
  visualization_msgs
  blue_hardware_drivers
  blue_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hardstop_calibrate src/hardstop_calibrate.cpp)
target_link_libraries(hardstop_calibrate ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS hardstop_calibrate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

