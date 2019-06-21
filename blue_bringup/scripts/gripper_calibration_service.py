#!/usr/bin/env python

"""This node provides a service to calibrate the grippers."""

import rospy
import actionlib
import numpy as np
from blue_msgs.srv import GripperPositionCalibration
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import LoadController
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Trigger, TriggerResponse
from blue_msgs.msg import MotorState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal

def get_gripper_actuator_position():
    motor_state_msg = rospy.wait_for_message("blue_hardware/motor_states", MotorState)
    for i, n in enumerate(motor_state_msg.name):
        if n == motor_names[-1]:
            return motor_state_msg.position[i]

def handle_load_controllers():
    unload_list = []
    load_controllers_list = ['blue_controllers/gripper_torque_controller']
    loaded_controllers = list_controllers()
    for controller in load_controllers_list:
        load = True
        for loaded in loaded_controllers.controller:
            if loaded.name == controller:
                # controller already loaded
                if loaded.state == "running":
                    return False
                load = False
                break
        if load:
            # controller not loaded yet
            load_controller(controller)
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
        cmd.data = [3.0]
        for i in range(5):
            cmd_pub.publish(cmd)
            rospy.sleep(0.1)

        # Wait until gripper is closed/stops moving
        gripper_position_previous = get_gripper_actuator_position()
        gripper_position_current = gripper_position_previous + 10
        while not rospy.is_shutdown() and np.abs(gripper_position_current - gripper_position_previous) > 1e-5:
            cmd_pub.publish(cmd)
            gripper_position_current = gripper_position_previous
            gripper_position_previous = get_gripper_actuator_position()

        # Calibrate gripper to closed position
        # TODO: don't hardcode this value -- read from yaml? URDF?
        response = gripper_position_calibration(1.05)
        if response.success:
            rospy.loginfo("Gripper calibration succeeded!")
        else:
            rospy.logerr("Gripper calibration failed!")
            return TriggerResponse(False, "Fail")

        # Stop applying force
        cmd.data = [0.0]
        for i in range(5):
            cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        rospy.loginfo("Gripper calibration succeeded!")

        # Switch out controllers
        gripper_controller_response = switch_controller([], ['blue_controllers/gripper_torque_controller'], 2)
        # Unload controllers that were loaded
        for controller in unload_list:
            unload_controller(controller)


    except rospy.ServiceException as e:
        rospy.logerr("Gripper calibration failed: %s" % e)
        return TriggerResponse(False, "Fail")

    return TriggerResponse(True, "Success")

if __name__ == "__main__":
    rospy.init_node("simple_startup_calibration")

    # Get list of motor names from parameter server
    motor_names = rospy.get_param("blue_hardware/motor_names")

    # Wait for calibration service and controller manager service
    rospy.loginfo("Waiting for calibration service and controller manager...")
    rospy.wait_for_service('blue_hardware/gripper_position_calibration')
    rospy.wait_for_service('controller_manager/switch_controller')

    # Controller Services
    load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
    unload_controller = rospy.ServiceProxy('controller_manager/unload_controller', LoadController)
    list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)

    # Initialize gripper calibration procedure
    gripper_position_calibration = rospy.ServiceProxy('blue_hardware/gripper_position_calibration', GripperPositionCalibration)
    switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
    cmd_pub = rospy.Publisher("blue_controllers/gripper_torque_controller/command", Float64MultiArray, queue_size=1)

    rospy.Service('calibrate_gripper', Trigger, handle_calibration_service)
    rospy.spin()
