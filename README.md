# Koko Core Software
This repository provides the core software used to run a Koko robot arm with ROS.

The software stack is set up as a ROS metapackage, which organizes our codebase into five individual packages:
- **koko_bringup**: launch and configuration files used to start up the processes needed for our robot to run
- **koko_controller_manager**: higher level control code, built off the [ros_control framework](http://wiki.ros.org/ros_control)
- **koko_controllers**: source code for our custom controller plugins (such as cartesian pose control), which can be dynamically started and stopped by the controller manager
- **koko_descriptions**: physical descriptions of our robot, in the form of URDF files and associated 3D models
- **koko_hardware_drivers**: lower-level code for communicating with hardware (ie motor drivers) and providing a ROS interface to them

-----

Manual development environment setup (Ubuntu 16.0.4 w/ ROS Kinetic):

```bash
mkdir -p ~/koko_ws/src && cd "$_"
catkin_init_workspace
git clone https://github.com/brentyi/koko.git
rosdep install --from-paths src --ignore-src -r -y
catkin_make install
```
