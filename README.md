<<<<<<< HEAD
# Koko Core Software
This repository provides the core software needed to run a Koko robot arm with ROS.

The software stack is set up as a ROS metapackage, which organizes our codebase into five individual packages:
- **koko_bringup**
  - Launch and configuration files used to start up the processes needed for our robot to run
- **koko_controller_manager**
  - Higher level control code, built off the [ros_control framework](http://wiki.ros.org/ros_control)
  - Contains our controller manager, which provides infrastructure for dynamically starting and stopping different types of controllers: end effector pose control, joint position control, velocity control, etc.
  - Also contains the controller manager's hardware interface, which acts as an abstraction barrier between our joint and motor messages
- **koko_controllers**
  - Source code for our custom controller plugins
- **koko_hardware_drivers**
  - Lower-level code for communicating with hardware (ie motor drivers) and providing a ROS interface to them
- **koko_descriptions**
  - Physical descriptions of our robot, in the form of URDF files and associated 3D models

-----

Manual development environment setup (Ubuntu 16.0.4 w/ ROS Kinetic):

```bash
mkdir -p ~/koko_ws/src && cd "$_"
catkin_init_workspace
git clone https://github.com/brentyi/koko.git
rosdep install --from-paths src --ignore-src -r -y
catkin_make install
```
