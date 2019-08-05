#!/usr/bin/env python
import time
import sys
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from blue_msgs.msg import MotorState

class Arm:
    def __init__(self, side):
        self.motor_pos = [0.0 for i in range(8)]
        self.motor_names = rospy.get_param(side + "_arm/blue_hardware/motor_names")
        rospy.Subscriber(side + "_arm/blue_hardware/motor_states", MotorState, self.update_motors)

    def update_motors(self, msg):
        for i, n in enumerate(msg.name):
            for j, j_name in enumerate(self.motor_names):
                if j_name == n:
                    self.motor_pos[j] = msg.position[i]

    def get_motor_state(self):
        return self.motor_pos

gr1 = 7.1875
gr2 = 8.2444852941

if __name__ == '__main__':
    rospy.init_node('Calibration', anonymous=True)

    rot1 = None
    rot2 = None
    side = None

    if len(sys.argv) == 3:
        side    = sys.argv[1]
        version = sys.argv[2]

        if side ==  'left':
            rot2 = np.pi/2
        elif side ==  'right':
            rot2 = -np.pi/2
        else:
            rospy.logerr("Please specify left or right for arm side (use right for single arm setup)")
            exit()

        if version ==  '1':
            rot1 = -2.189
        elif version ==  '2':
            rot1 = -2.310
        else:
            rospy.logerr("Please specify a 1 or 2 for arm version")
            exit()

    else:
        rospy.logerr("usage:")
        rospy.logerr("rosrun blue_bringup calibrate_joint_offsets.py <side (left or right)> <version (1 or 2)>")
        exit()

    arm = Arm(side)

    raw_input("Press enter to save base state")
    ms = arm.get_motor_state()
    a0 = ms[0]

    raw_input("Press enter to save first link state")
    ms = arm.get_motor_state()
    a1 = ms[1] +  rot1 * gr1 + rot2 * gr2
    a2 = ms[2] + -rot1 * gr1 + rot2 * gr2

    raw_input("Press enter to save second link state")
    ms = arm.get_motor_state()
    # Rotate elbow roll in the opposite direction as the shoulder roll
    a3 = ms[3] +  rot1 * gr1 - rot2 * gr2
    a4 = ms[4] + -rot1 * gr1 - rot2 * gr2

    raw_input("Press enter to save third link state")
    # The wrist uses the version 1 differential coupling
    ms = arm.get_motor_state()
    a5 = ms[5] + -2.189 * gr1
    a6 = ms[6] +  2.189 * gr1

    actuators = [a0, a1, a2, a3, a4, a5, a6]
    np.set_printoptions(precision=4)
    print("Save the following to the yaml configuration file")
    print(np.array2string(np.array(actuators), separator=', '))
