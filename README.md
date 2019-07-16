# Blue Core Software
This repository provides all of the mid-level software needed to run a Blue robot arm with ROS.

The stack is set up as a ROS metapackage, which organizes our codebase into the following packages:
- **blue_bringup**
  - Launch files, configurations, and scripts used to start up the robot
- **blue_controller_manager**
  - Higher level control code, built off the [ros_control framework](http://wiki.ros.org/ros_control)
  - Provides infrastructure for dynamically starting and stopping different types of controllers: end effector pose control, joint position control, velocity control, etc.
- **blue_hardware_interface**
  - Abstraction barrier between joint messages and actuator messages (see [ros_control](http://wiki.ros.org/ros_control))
- **blue_controllers**
  - Source code for our custom controller plugins
- **blue_hardware_drivers**
  - Lower-level code for communicating with hardware (ie motor drivers) and providing a ROS interface to them
- **blue_descriptions**
  - Physical descriptions of our robot, in the form of URDF files and associated 3D models
- **blue_msgs**
  - Message types and service descriptions

-----

## How do I set up the computer to run my arm?

- Install Ubuntu 16.0.4 or 18.0.4
- Install ROS [Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (16.0.4) or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (18.0.4)
  - For step 1.4, use Desktop-Full if unsure
- Create a workspace:
  ```bash
  mkdir -p ~/blue_ws/src && cd "$_"
  ```
- Clone the code:
  ```bash
  git clone https://github.com/berkeleyopenarms/blue_core.git
  ```
- Install dependencies:
  ```bash
  cd ~/blue_ws
  rosdep install --from-paths src --ignore-src -r -y
  ```
- Build:
  ```bash
  catkin_make install
  ```
- Source:
  ```bash
  echo "source ~/blue_ws/devel/setup.bash" >> ~/.bashrc
  source ~/blue_ws/devel/setup.bash
  ```
- Set up user permissions (for serial port access -- you'll need to log out and back in for this take effect):
  ```bash
  sudo addgroup $USER dialout
  ```
- Set up a configuration file (see the [blue_configs](https://github.com/berkeleyopenarms/blue_configs) repo for examples)

- Proceed to setup the arm with power supply and USB adapter ("Electrical Setup" in Quick Start Guide)

-----

## The robot is now turned on, connected via USB, and in the proper startup position -- how do I run the control stack?

After running the above setup steps, the following will boot the arm and put it into gravity compensation mode:

- For a right arm (default setup):
  ```bash
  roslaunch blue_bringup right.launch param_file:=blue_params.yaml
  ```
- For a left arm (yaml file will need to be namespaced relative to `left_arm` instead of `right_arm`):
  ```bash
  roslaunch blue_bringup left.launch param_file:=blue_params.yaml
  ```
-----

## How do I calibrate the gripper?

At startup, the software stack assumes the gripper is open. If Blue was started with the gripper open, then no additional steps are needed!

However, if the gripper is started in any other position, then an optional gripper calibration service should be called before using the gripper. This service will automatically determine the gripper position by apply a closing torque and detecting when the gripper has fully closed.

From the command line:
- ```bash
  rosservice call /<left or right>_arm/calibrate_gripper "{}"
  ```
- Gripper controllers should not be started when this service is called.
- This functionality is also supported by [blue_interface](https://github.com/berkeleyopenarms/blue_interface)

-----
## Experimental two arm

- For the experimental two arm setup, the .yaml file should contain parameters for both the left and right arms:
  ```
  right_arm/blue_hardware:
        serial_port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A506MP4W-if00-port0
        motor_ids: [40, 76, 65, 70, 77, 20, 21, 52]
        simple_startup_angles: [-0.785398, -2.19, -1.570796, 0.0, 1.570796, -0.23, 0.0, 0.0]
  left_arm/blue_hardware:
        serial_port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI057K87-if00-port0
        motor_ids: [30, 62, 69, 16, 33, 51, 50, 55]
        simple_startup_angles: [-0.785398, -2.19, -1.570796, 0.0, 1.570796, -0.23, 0.0, 0.0]
  ```
- To run:
  ```bash
  roslaunch blue_bringup full.launch param_file:=blue_params.yaml
  ```

-----
## rviz

```bash
roslaunch blue_bringup rviz.launch
```
