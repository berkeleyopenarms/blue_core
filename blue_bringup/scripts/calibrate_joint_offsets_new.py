#!/usr/bin/env python

import numpy as np


def old(actuator_angles):

    side = "right"
    version = "2"

    gr1 = 7.1875
    gr2 = 8.2444852941

    if side == 'left':
        rot2 = np.pi / 2
    elif side == 'right':
        rot2 = -np.pi / 2

    if version == '1':
        rot1 = -2.189
    elif version == '2':
        rot1 = -2.310

    ms = actuator_angles
    a0 = ms[0]
    a1 = ms[1] + rot1 * gr1 + rot2 * gr2
    a2 = ms[2] + -rot1 * gr1 + rot2 * gr2
    a3 = ms[3] + rot1 * gr1 - rot2 * gr2
    a4 = ms[4] + -rot1 * gr1 - rot2 * gr2
    a5 = ms[5] + -2.189 * gr1
    a6 = ms[6] + 2.189 * gr1
    actuators = [a0, a1, a2, a3, a4, a5, a6]

    return np.array(actuators)


def new(actuator_angles):

    # Helper function for propagating joint angles => actuator angles
    def actuator_angles_from_joint_angles(joint_angles):

        # Transmission helpers
        def propagate_through_base_roll(base_roll_angle):
            base_roll_ratio = 7.1875
            return base_roll_ratio * base_roll_angle

        def propagate_through_differential(lift_angle, roll_angle):
            lift_ratio = 7.1875
            roll_ratio = 8.2444852941
            left_actuator_angle = -lift_angle * lift_ratio + roll_angle * roll_ratio
            right_actuator_angle = lift_angle * lift_ratio + roll_angle * roll_ratio
            return (left_actuator_angle, right_actuator_angle)

        # Propagate and return!
        actuator_angles = np.hstack((
            propagate_through_base_roll(joint_angles[0]),
            propagate_through_differential(joint_angles[1], joint_angles[2]),
            propagate_through_differential(joint_angles[3], joint_angles[4]),
            propagate_through_differential(joint_angles[5], joint_angles[6]),
        ))
        assert(actuator_angles.shape == (7,))
        return actuator_angles

    # Determine robot calibration position
    side = "right"
    version = "2"

    roll_angle = {
        "left": -np.pi / 2,
        "right": np.pi / 2
    }[side]
    lift_angle = {
        "1": -2.189,
        "2": -2.310,
    }[version]
    gripper_lift_angle = -2.189

    calibration_joint_positions = [
        0,  # Base roll
        lift_angle,
        roll_angle,
        lift_angle,
        -roll_angle,
        gripper_lift_angle,
        0  # Gripper roll
    ]

    return actuator_angles - \
        actuator_angles_from_joint_angles(calibration_joint_positions)


for _ in range(10):
    actuator_angles = np.random.random(7) * 20 - 10
    assert(np.linalg.norm(old(actuator_angles) - new(actuator_angles)) < 1e-5)
