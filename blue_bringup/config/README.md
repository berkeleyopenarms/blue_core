### Parameter File Documentation
The robot parameters file specifies the important hardware details of the blue arm.

* joint_names:
  * specifies the names of the joints, which corresponds to the URDF in the blue_descriptions ROS package.
* baselink
  * specifies the name of the starting link of the kinematic chain
* endlink
  * specifies the name of the ending link of the kinematic chain
* accel_links
  * specifies the name of the links of the accelerometers. The accelerometer gravity vectors are publish in this links frame.
* differential_pairs
  * specifies which motors pair together to for a differential. This must be even as the numbers in this list are considered as pairs.
* gear_ratios
  * specifies the gear ratio of each each transmission
* joint_torque_directions
  * specifies the torque direction of the motor with respect to the encoder direction
* id_torque_gains
  * specifies the multiplier on the inverse dynamics torques. Can be used to tune gravity compensation
* softstop_min_angles
  * min joint limits of software joint stop
* softstop_max_angles
  * max joint limits of software joint stop
* softstop_tolerance
  * tolerance from the softstop limits where a counter torque is applied (for protection of the robot against its physical hardstops)
* softstop_torque_limit
  * gain on softstop torque


* motor_names:
  * names of the motors, in the order they are communicated to. Convention for the links is left then right.
* motor_current_limits
  * hard current limits on the motors
* current_to_torque_ratios
  * conversion between motor torque and motor current, taken from motor properties
* posture_target
  * used to as a control parameter for posture control
* posture_weights
  * used to as a control parameter for posture control
