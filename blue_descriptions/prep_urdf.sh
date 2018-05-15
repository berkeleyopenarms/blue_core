#!/bin/bash

# For converting a new URDF from solidworks to be ready to launch with ROS
# Usage
# ./prep_urdf.sh $folder name
file_folder=$1
mv $file_folder/urdf $file_folder/robots
sed -i 's/False/True/g' $file_folder/launch/display.launch
cp blue_description_sevendof_kin_old/urdf.rviz $file_folder
sed -i 's:<maintainer email="me2email.com" />:<maintainer email="lxd20@todo.todo">lxd20</maintainer>:g' $file_folder/package.xml
