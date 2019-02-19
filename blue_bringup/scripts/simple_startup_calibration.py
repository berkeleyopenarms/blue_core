#!/usr/bin/env python

"""This node should run at startup, and sets the initial joint angles to some hardcoded values."""

import rospy
from blue_msgs.srv import JointStartupCalibration

if __name__ == "__main__":
    rospy.init_node("simple_startup_calibration")

    # Read startup angles from parameter server
    rospy.loginfo("Reading desired joint angles...")
    startup_positions = rospy.get_param("blue_hardware/simple_startup_angles")
    disable_snap = rospy.get_param("blue_hardware/disable_snap")

    # Wait for calibration service to come up
    rospy.loginfo("Waiting for calibration service...")
    rospy.wait_for_service('blue_hardware/joint_startup_calibration')

    # Calibrate joints with startup angles
    rospy.loginfo("Starting calibration...")
    try:
        joint_startup_calibration = rospy.ServiceProxy('blue_hardware/joint_startup_calibration', JointStartupCalibration)
        response = joint_startup_calibration(startup_positions, disable_snap)

        if response.success:
            rospy.loginfo("Joint startup calibration succeeded!")
        else:
            rospy.logerr("Joint startup calibration failed!")

    except rospy.ServiceException as e:
        rospy.logerr("Joint startup calibration failed: %s" % e)
