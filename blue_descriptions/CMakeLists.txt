cmake_minimum_required(VERSION 2.8.3)

project(blue_descriptions)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS xacro)

find_package(roslaunch)

foreach(dir config launch meshes robots)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
