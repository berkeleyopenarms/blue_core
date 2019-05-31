#!/usr/bin/env python

"""This node should run at startup, and sets the initial joint angles to some hardcoded values."""

import rospy
from blue_msgs.srv import JointStartupCalibration
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import LoadController
from blue_msgs.msg import MotorState
from std_msgs.msg import Float64MultiArray

def update_motors(motor_msg):
    global gripper_state
    global first_update
    global motor_names
    for i, n in enumerate(motor_msg.name):
        if n == motor_names[-1]:
            gripper_state = motor_msg.position[i]
            rospy.logerr(gripper_state)
            break
    first_update = True


if __name__ == "__main__":
    global gripper_state
    global first_update
    global motor_names
    rospy.init_node("simple_startup_calibration")
    rate = rospy.Rate(10)

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

        # gripper calibration procedure
        # switch to torque controller
        load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
        switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

        load_controller('blue_controllers/gripper_torque_controller')
        gripper_controller_response = switch_controller(['blue_controllers/gripper_torque_controller'], [], 2)

        # set subscribers and publishers
        first_update = False

        rospy.Subscriber("blue_hardware/motor_states", MotorState, update_motors)

        cmd_pub = rospy.Publisher("blue_controllers/gripper_torque_controller/command", Float64MultiArray, queue_size=1)
        cmd = Float64MultiArray()
        cmd.data = [7.5]
        # apply positive torque
        for i in range(5):
            cmd_pub.publish(cmd)
            rate.sleep()

        while not rospy.is_shutdown() or not first_update:
            rate.sleep()

        current_position = gripper_state + 10
        # busy wait until gripper is closed
        while not rospy.is_shutdown() or np.abs(curent_position - gripper_state) < 1e-4:
            # apply positive torque
            cmd_pub.publish(cmd)
            # gripper still closing
            current_position = gripper_state
            rate.sleep()
        rospy.logerr("gripper torque command")

        # set simple calibration angles again to calibrate gripper
        joint_startup_calibration = rospy.ServiceProxy('blue_hardware/joint_startup_calibration', JointStartupCalibration)
        response = joint_startup_calibration(startup_positions, disable_snap)

        if response.success:
            rospy.loginfo("Joint startup calibration succeeded!")
        else:
            rospy.logerr("Joint startup calibration failed!")



    except rospy.ServiceException as e:
        rospy.logerr("Joint startup calibration failed: %s" % e)
