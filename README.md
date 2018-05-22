# Blue Core Software
This repository provides the core software needed to run a Blue robot arm with ROS.

The software stack is set up as a ROS metapackage, which organizes our codebase into five individual packages:
- **blue_bringup**
  - Launch and configuration files used to start up the processes needed for our robot to run
- **blue_controller_manager**
  - Higher level control code, built off the [ros_control framework](http://wiki.ros.org/ros_control)
  - Contains our controller manager, which provides infrastructure for dynamically starting and stopping different types of controllers: end effector pose control, joint position control, velocity control, etc.
  - Also contains the controller manager's hardware interface, which acts as an abstraction barrier between our joint and motor messages
- **blue_controllers**
  - Source code for our custom controller plugins
- **blue_hardware_drivers**
  - Lower-level code for communicating with hardware (ie motor drivers) and providing a ROS interface to them
- **blue_descriptions**
  - Physical descriptions of our robot, in the form of URDF files and associated 3D models

-----

## How do I set up a development environment?

- Install Ubuntu 16.0.4
- [Install ROS Kinetic]
- Create a workspace:
  ```bash
  mkdir -p ~/blue_ws/src && cd "$_"
  catkin_init_workspace
  ```
- Clone the code:
  ```bash
  git clone https://github.com/berkeley-open-robotics/blue_core.git
  ```
- Install dependencies:
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```
- Build:
  ```bash
  cd ~/blue_ws
  catkin_make install
  ```

-----

## I have a robot turned on and connected to my computer -- how do I run the control stack?

With your workspace's `devel/setup.bash` script sourced:
- For a left arm:
  ```bash
  roslaunch blue_bringup left.launch
  ``` 
- For a right arm:
  ```bash
  roslaunch blue_bringup right.launch
  ``` 
- For the (experimental) two-arm setup:
  ```bash
  roslaunch blue_bringup full.launch
  ```
  
For different arms, there are a handful of configuration values that currently still need to be changed in the `koko_bringup/config/robot_parameters_*.yaml` files. Notably:
- Serial port
- Motor driver IDs
