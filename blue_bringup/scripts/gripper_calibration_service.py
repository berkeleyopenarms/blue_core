#!/usr/bin/env python

"""This node should run at startup, and sets the initial joint angles to some hardcoded values."""

import rospy
import numpy as np
import actionlib
from blue_msgs.srv import JointStartupCalibration
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import LoadController
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from blue_msgs.msg import MotorState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

def update_motors(motor_msg):
    global gripper_state
    global motor_names
    for i, n in enumerate(motor_msg.name):
        if n == motor_names[-1]:
            gripper_state = motor_msg.position[i]
            break

def handle_calibration_service():
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
        load_controller('blue_controllers/gripper_controller')
        gripper_controller_response = switch_controller(['blue_controllers/gripper_torque_controller'], [], 2)

        cmd = Float64MultiArray()
        cmd.data = [2.0]
        # apply positive torque
        for i in range(5):
            cmd_pub.publish(cmd)
            rate.sleep()
        rospy.wait_for_message("blue_hardware/motor_states", MotorState)

        current_position = gripper_state + 10
        # busy wait until gripper is closed
        while not rospy.is_shutdown() and np.abs(current_position - gripper_state) > 1e-5:
            # apply positive torque
            # gripper still closing
            cmd_pub.publish(cmd)
            current_position = gripper_state
            rospy.wait_for_message("blue_hardware/motor_states", MotorState)

        # set simple calibration angles again to calibrate gripper
        response = joint_startup_calibration(startup_positions, disable_snap)
        if response.success:
            rospy.loginfo("Joint startup calibration succeeded!")
        else:
            rospy.logerr("Joint startup calibration failed!")

        cmd.data = [0.0]
        # apply positive torque
        for i in range(5):
            cmd_pub.publish(cmd)
            rate.sleep()

        # switch out controllers to reset position
        gripper_controller_response = switch_controller(['blue_controllers/gripper_controller'], ['blue_controllers/gripper_torque_controller'], 2)

        client = actionlib.SimpleActionClient(
            "blue_controllers/gripper_controller/gripper_cmd",
            GripperCommandAction,
        )

        # Wait 10 Seconds for the gripper action server to start or exit
        if not client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)

        goal = GripperCommandGoal()
        goal.command.position = 0.0
        goal.command.max_effort = 2.0
        client.send_goal(goal)
        client.wait_for_result()

        # clear gripper controllers
        gripper_controller_response = switch_controller([], ['blue_controllers/gripper_controller'], 2)
        rospy.loginfo("Gripper Calibration Succeeded!")

    except rospy.ServiceException as e:
        rospy.logerr("Joint startup calibration failed: %s" % e)
        return TriggerResponse(False, "Fail")
    return TriggerResponse(True, "Succsess")


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

    s = rospy.Service('calibrate_gripper', Trigger, handle_calibration_service)
    rospy.spin()
