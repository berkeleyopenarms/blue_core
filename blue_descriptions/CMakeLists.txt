cmake_minimum_required(VERSION 3.0.2)

project(blue_descriptions)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS xacro)

find_package(roslaunch)

foreach(dir launch meshes robots)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
