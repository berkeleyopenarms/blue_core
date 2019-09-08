#!/usr/bin/env python
import sys
import numpy as np
import rospy
from blue_msgs.msg import MotorState


def get_actuator_positions(side):
    # Helper function for reading actuator positions

    motor_names = rospy.get_param(
        "{}_arm/blue_hardware/motor_names".format(side))
    # Should include 7 joint motors and 1 gripper motor
    assert(len(motor_names) == 8)

    motor_msg = rospy.wait_for_message(
        "{}_arm/blue_hardware/motor_states".format(side),
        MotorState)

    # Retrieve actuator positions for motors only
    motor_positions = []
    for name in motor_names[:7]:
        motor_index = motor_msg.name.index(name)
        motor_positions.append(motor_msg.position[motor_index])

    return motor_positions


def actuator_angles_from_joint_angles(joint_angles, side):
    # Helper function for propagating joint angles => actuator angles

    # Transmission helpers
    def backpropagate_simple_transmission(angle, ratio):
        return ratio * angle

    def backpropagate_differential_transmission(
            lift_angle, roll_angle, lift_ratio, roll_ratio):

        left_actuator_angle = -lift_angle * lift_ratio + roll_angle * roll_ratio
        right_actuator_angle = lift_angle * lift_ratio + roll_angle * roll_ratio
        return (left_actuator_angle, right_actuator_angle)

    # Get gear ratios and compute actuator angles
    gear_ratios = rospy.get_param(
        "{}_arm/blue_hardware/gear_ratios".format(side))
    actuator_angles = []

    # Base transmission
    actuator_angles.append(
        backpropagate_simple_transmission(
            angle=joint_angles[0],
            ratio=gear_ratios[0]
        )
    )

    # Differential transmissions between links
    for i in range(3):
        index0 = 1 + i * 2
        index1 = 2 + i * 2
        actuator_angles.extend(
            backpropagate_differential_transmission(
                lift_angle=joint_angles[index0],
                roll_angle=joint_angles[index1],
                lift_ratio=gear_ratios[index0],
                roll_ratio=gear_ratios[index1]
            )
        )

    # Convert to numpy array and return
    actuator_angles = np.array(actuator_angles)
    assert(actuator_angles.shape == (7,))
    return actuator_angles


if __name__ == '__main__':
    rospy.init_node('blue_joint_offset_calibration', anonymous=False)

    # Check argument count
    if len(sys.argv) != 3:
        # Invalid argument count => display error and quit
        rospy.logerr("usage:")
        rospy.logerr(
            "rosrun blue_bringup calibrate_joint_offsets.py <side (left or right)> <version (1 or 2)>")
        exit()

    # Parse arguments
    side = sys.argv[1]
    version = sys.argv[2]

    # Check argument validity
    if side not in ("left", "right"):
        rospy.logerr(
            "Please specify left or right for arm side (use right for single arm setup)")
        exit()
    if version not in ("1", "2"):
        rospy.logerr("Please specify 1 or 2 for arm version")
        exit()

    # Determine calibration position based on side and version
    roll_angle = {
        "left": -np.pi / 2,
        "right": np.pi / 2
    }[side]
    lift_angle = {
        "1": -2.189,
        "2": -2.310,
    }[version]

    calibration_joint_positions = [
        0,  # Base roll
        lift_angle,
        roll_angle,
        lift_angle,
        -roll_angle,
        -2.189,  # Gripper lift angle -- last link is always a v1
        0  # Gripper roll
    ]

    # Ask user to step robot through calibration positions, and record
    # actuator states at each one
    actuator_angles = [0] * 7

    def save_positions(*actuator_indices):
        for i in actuator_indices:
            actuator_angles[i] = get_actuator_positions(side)[i]

    # Calibrate base joint
    raw_input("Press [Enter] when base joint is in calibration position")
    save_positions(0)

    # Calibrate shoulder joints
    raw_input("Press [Enter] when first link is in calibration position")
    save_positions(1, 2)

    # Calibrate elbow joints
    raw_input("Press [Enter] when second link is in calibration position")
    save_positions(3, 4)

    # Calibrate wrist joints
    raw_input("Press [Enter] when third link is in calibration position")
    save_positions(5, 6)

    # Computer actuator zeros and print
    actuator_zeros = actuator_angles - \
        actuator_angles_from_joint_angles(calibration_joint_positions, side)
    np.set_printoptions(precision=4)
    print("===============")
    print("Save the following actuator zeros to the robot-specific configuration file:")
    print(np.array2string(np.array(actuator_zeros), separator=', '))
