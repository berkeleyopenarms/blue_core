#!/usr/bin/env python

"""This node should run at startup, and sets the initial joint angles to some hardcoded values."""

import rospy
import numpy as np
from blue_msgs.srv import JointStartupCalibration
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import LoadController
from blue_msgs.msg import MotorState
from std_msgs.msg import Float64MultiArray

def update_motors(motor_msg):
    global gripper_state
    global motor_names
    for i, n in enumerate(motor_msg.name):
        if n == motor_names[-1]:
            gripper_state = motor_msg.position[i]
            # rospy.logerr(gripper_state)
            break


if __name__ == "__main__":
    global gripper_state
    global motor_names
    rospy.init_node("simple_startup_calibration")
    rate = rospy.Rate(5)

    # Read startup angles from parameter server
    rospy.loginfo("Reading desired joint angles...")
    startup_positions = rospy.get_param("blue_hardware/simple_startup_angles")
    motor_names = rospy.get_param("blue_hardware/motor_names")
    disable_snap = rospy.get_param("blue_hardware/disable_snap", False)

    # Wait for calibration service to come up
    rospy.loginfo("Waiting for calibration service...")
    rospy.wait_for_service('blue_hardware/joint_startup_calibration')
    rospy.wait_for_service('controller_manager/switch_controller')

    # Calibrate joints with startup angles
    rospy.loginfo("Starting calibration...")
    try:
        # set simple calibration angles
        joint_startup_calibration = rospy.ServiceProxy('blue_hardware/joint_startup_calibration', JointStartupCalibration)
        response = joint_startup_calibration(startup_positions, disable_snap)

        if response.success:
            rospy.loginfo("Joint startup calibration succeeded!")
        else:
            rospy.logerr("Joint startup calibration failed!")

        # gripper calibration procedure, first switch to torque controller
        load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
        switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        rospy.Subscriber("blue_hardware/motor_states", MotorState, update_motors)
        cmd_pub = rospy.Publisher("blue_controllers/gripper_torque_controller/command", Float64MultiArray, queue_size=1)

        load_controller('blue_controllers/gripper_torque_controller')
        gripper_controller_response = switch_controller(['blue_controllers/gripper_torque_controller'], [], 2)

        cmd = Float64MultiArray()
        cmd.data = [3.5]
        # apply positive torque
        for i in range(5):
            cmd_pub.publish(cmd)
            rate.sleep()
        rospy.wait_for_message("blue_hardware/motor_states", MotorState)

        current_position = gripper_state + 10
        # busy wait until gripper is closed
        rospy.logerr(np.abs(current_position - gripper_state))
        while not rospy.is_shutdown() and np.abs(current_position - gripper_state) > 1e-5:
            rospy.logerr(np.abs(current_position - gripper_state))
            # apply positive torque
            # gripper still closing
            cmd_pub.publish(cmd)
            current_position = gripper_state
            rospy.wait_for_message("blue_hardware/motor_states", MotorState)
            rospy.wait_for_message("blue_hardware/motor_states", MotorState)
            rospy.logerr(np.abs(current_position - gripper_state))

        for i in range(5):
            cmd_pub.publish(cmd)
            rate.sleep()

        # set simple calibration angles again to calibrate gripper
        response = joint_startup_calibration(startup_positions, disable_snap)

        if response.success:
            rospy.loginfo("Joint startup calibration succeeded!")
        else:
            rospy.logerr("Joint startup calibration failed!")

        cmd.data = [0.0]
        # apply positive torque
        for i in range(10):
            cmd_pub.publish(cmd)
            rate.sleep()

        gripper_controller_response = switch_controller([], [], 2)



    except rospy.ServiceException as e:
        rospy.logerr("Joint startup calibration failed: %s" % e)
