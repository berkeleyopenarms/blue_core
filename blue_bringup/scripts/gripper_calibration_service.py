#!/usr/bin/env python

"""This node provides a service to calibrate the grippers."""

import rospy
import numpy as np
import actionlib
from blue_msgs.srv import GripperPositionCalibration
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import LoadController
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from blue_msgs.msg import MotorState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

def get_gripper_state(motor_names):
    motor_state_msg = rospy.wait_for_message("blue_hardware/motor_states", MotorState)
    for i, n in enumerate(motor_state_msg.name):
        if n == motor_names[-1]:
            return motor_state_msg.position[i]

def handle_load_controllers():
    unload_list = []
    load_controllers_list = ['blue_controllers/gripper_torque_controller', 'blue_controllers/gripper_controller']
    loaded_controllers    = list_controllers()
    rospy.logerr(loaded_controllers.controller)
    for controller in load_controllers_list:
        load = True
        for loaded in loaded_controllers.controller:
            rospy.logerr(loaded.name)
            rospy.logerr(controller)
            rospy.logerr(controller)

            if loaded.name == controller:
                # controller already loaded
                if loaded.state == "running":
                    return False
                load = False
                break
        if load:
            # controller not loaded yet
            controller_response = load_controller(controller)
            unload_list.append(controller)
    return unload_list


def handle_calibration_service(input_request):
    # Calibrate joints with startup angles
    rospy.loginfo("Starting calibration...")
    try:
        unload_list = handle_load_controllers()
        if unload_list == False:
            return TriggerResponse(False, "Gripper Calibration Failed. Gripper resource must be free.")
        gripper_controller_response = switch_controller(['blue_controllers/gripper_torque_controller'], [], 2)

        # Close the gripper
        cmd = Float64MultiArray()
        cmd.data = [5.0]
        for i in range(5):
            cmd_pub.publish(cmd)
            rospy.sleep(0.1)

        # Wait until gripper is closed/stops moving
        gripper_state = get_gripper_state(motor_names)
        current_position = gripper_state + 10
        while not rospy.is_shutdown() and np.abs(current_position - gripper_state) > 1e-5:
            cmd_pub.publish(cmd)
            current_position = gripper_state
            gripper_state = get_gripper_state(motor_names)

        # Stop applying force
        cmd.data = [0.0]
        for i in range(5):
            cmd_pub.publish(cmd)
            rospy.sleep(0.1)

        # Calibrate gripper to closed position
        # TODO: don't hardcode this value -- read from yaml? URDF?
        response = gripper_position_calibration(1.0)

        if response.success:
            rospy.loginfo("Gripper position calibration succeeded!")
        else:
            rospy.logerr("Gripper position calibration failed!")

        # Switch out controllers to reset position
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
        goal.command.max_effort = 6.0
        client.send_goal(goal)
        client.wait_for_result()

        # Clear gripper controllers
        gripper_controller_response = switch_controller([], ['blue_controllers/gripper_controller'], 2)
        rospy.loginfo("Gripper Calibration Succeeded!")

        # Unload controllers that were loaded
        for controller in unload_list:
            unload_controller(controller)


    except rospy.ServiceException as e:
        rospy.logerr("Joint startup calibration failed: %s" % e)
        return TriggerResponse(False, "Fail")

    return TriggerResponse(True, "Succsess")

if __name__ == "__main__":

    rospy.init_node("simple_startup_calibration")

    # Read startup angles from parameter server
    rospy.loginfo("Reading desired joint angles...")
    motor_names = rospy.get_param("blue_hardware/motor_names")

    # Wait for calibration service to come up
    rospy.loginfo("Waiting for calibration service...")
    rospy.wait_for_service('blue_hardware/gripper_position_calibration')
    rospy.wait_for_service('controller_manager/switch_controller')

    # Controller Services
    load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
    unload_controller = rospy.ServiceProxy('controller_manager/unload_controller', LoadController)
    list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)

    # gripper calibration procedure, first switch to torque controller
    gripper_position_calibration = rospy.ServiceProxy('blue_hardware/gripper_position_calibration', GripperPositionCalibration)
    switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
    cmd_pub = rospy.Publisher("blue_controllers/gripper_torque_controller/command", Float64MultiArray, queue_size=1)

    rospy.Service('calibrate_gripper', Trigger, handle_calibration_service)
    rospy.spin()
