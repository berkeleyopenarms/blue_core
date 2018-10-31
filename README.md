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
- **blue_msgs**
  - Message types and service descriptions

-----

## How do I set up the computer to run my arm?

- Install Ubuntu 16.0.4
- [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  - Start from 1.2 to 1.7, just copy paste into terminal 
  - For step 1.4, use bare bones unless you plan on developing using ROS visualization tools
- Create a workspace:
  ```bash
  mkdir -p ~/blue_ws/src && cd "$_"
  ```
- Clone the code:
  ```bash
  git clone https://github.com/berkeley-open-robotics/blue_core.git
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
- Setup user permissions (you'll need to log out and back in for this take effect):
  ```bash
  sudo addgroup $USER dialout
  ```
- There are a handful of configuration that are specific to your arm that you need to set
  ```bash
  cd ~/.ros
  touch blue_params.yaml
  ```
- In your newly created ```blue_params.yaml``` file add the following under the `right_arm/blue_hardware` namespace
  - `serial_port`
    - path to the serial port your arm is connected to
  - `motor_ids`
    - a list of the 8 motor ids, starting from the base to the gripper
  - `simple_startup_angles`
    - the intial starting joint angles of your robot upon startup
- To get you started, here is what our blue_params.yaml file looks like:
  ```
  right_arm/blue_hardware:
        serial_port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A506MP4W-if00-port0
        motor_ids: [40, 76, 65, 70, 77, 20, 21, 52]
        simple_startup_angles: [-0.6647, -2.1294, 0.8929, -2.1951, -2.0915, -0.5770, 0.0274, 0.0]
  ```

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
