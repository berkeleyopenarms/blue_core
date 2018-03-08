#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from koko_hardware_drivers.msg import MotorState

class Link:
    def __init__(self):
        self.lift_offset = 0.605
        p_constants = [15.0, 15.0]
        d_constants = [1.0, 1.0]

        rospy.init_node('Calibration', anonymous=True)

        rate = rospy.Rate(200)

        self.p_publisher = rospy.Publisher("/p_terms", Float64MultiArray, queue_size=1)
        self.d_publisher = rospy.Publisher("/d_terms", Float64MultiArray, queue_size=1)

        self.cmd = rospy.Publisher("/koko_controllers/joint_positions_controller/command", Float64MultiArray, queue_size=1)
        self.pos = [0.0, 0.0]
        self.motor_pos = [0.0, 0.0]

        self.set_pd(p_constants, d_constants)
        rospy.Subscriber("/joint_states", JointState, self.update_joints)
        rospy.Subscriber("/koko_hardware/motor_states", MotorState, self.update_motors)

    def set_pd(self, p, d):
        for i in range(2):
            p_terms = Float64MultiArray()
            d_terms = Float64MultiArray()

            p_terms.data = p
            d_terms.data = d

            self.p_publisher.publish(p_terms)
            self.d_publisher.publish(d_terms)
            rospy.logerr('finished publishing new PD constants')
            time.sleep(1)

    def set_pos(self, cmds):
        j_cmd = Float64MultiArray()
        j_cmd.data = cmds
        self.cmd.publish(j_cmd)

    def update_motors(self, joint_msg):
        for i, n in enumerate(joint_msg.name):
            if n == 'right_motor':
                self.motor_pos[0] = joint_msg.position[i]
            elif n == 'left_motor':
                self.motor_pos[1] = joint_msg.position[i]
            else:
                rospy.logerr('No Valid Joint Found')

    def update_joints(self, joint_msg):
        for i, n in enumerate(joint_msg.name):
            if n == 'lift_joint':
                self.pos[0] = joint_msg.position[i]
            elif n == 'roll_joint':
                self.pos[1] = joint_msg.position[i]
            else:
                rospy.logerr('No Valid Joint Found')

    def center_joint(self, j, _iter=1):
        max_pos_list = []
        min_pos_list = []
        for _ in range(_iter):
            min_pos_list.append(self.find_end(j, -1.0))
            max_pos_list.append(self.find_end(j, 1.0))

        max_pos = sum(max_pos_list) / float(_iter)
        min_pos = sum(min_pos_list) / float(_iter)
        return (max_pos + min_pos) / 2.0, max_pos, min_pos

    def find_end(self, j, direction):
        eps = 0.3 # joint position erro in radians
        curr_error = 0
        delta = 0.03
        command_pos = self.pos[:]
        while curr_error < eps and not rospy.is_shutdown():
            # rospy.logerr("{}".format(curr_error))
            command_pos[j] = command_pos[j] + direction *  delta
            self.set_pos(command_pos)
            curr_error = np.abs(command_pos[j] - self.pos[j])
            # rospy.logerr("{}".format(curr_error))
            rospy.sleep(0.095)

        rospy.sleep(1)
        rospy.loginfo('done finding the end')
        return self.pos[j]

if __name__ == '__main__':
    iterations = 5
    link = Link()
    rospy.loginfo('Link setup complete')
    # set to start up position
    link.set_pos([0.0, 0.0])
    rospy.sleep(0.5)
    rospy.loginfo('Set to zero')
    rospy.sleep(0.5)
    # centering roll link first
    center_roll, max_roll, min_roll = link.center_joint(1, _iter=iterations)
    link.set_pos([0.0, center_roll])

    rospy.sleep(2)
    p_constants = [70.0, 70.0]
    d_constants = [1.0, 1.0]
    link.set_pd(p_constants, d_constants)
    rospy.sleep(1)
    link.set_pos([0.0, center_roll])
    rospy.sleep(1)
    rospy.loginfo('Script Complete')

    p_constants = [10.0, 10.0]
    d_constants = [1.0, 1.0]
    link.set_pd(p_constants, d_constants)
    # finding end of roll
    lift_min_pos_list = []
    rospy.loginfo('Finding Roll Hardstop Complete')
    for _ in range(iterations):
        lift_min_pos_list.append(link.find_end(0, 1.0))
        link.set_pos([0.0, center_roll])
        rospy.sleep(0.1)
    lift_min = sum(lift_min_pos_list) / float(iterations)

    link.set_pos([lift_min - link.lift_offset, center_roll])
    rospy.sleep(0.4)

    link.set_pos([lift_min - link.lift_offset, center_roll])
    p_constants = [90.0, 90.0]
    d_constants = [1.0, 1.0]
    link.set_pd(p_constants, d_constants)
    print('calibration complete, set the folloing for the motor zero positions')
    print(link.motor_pos)

    print("Motor Position right")
    print(link.motor_pos[0])
    print(link.motor_pos[0]%np.pi)
    print("Motor Position left")
    print(link.motor_pos[1])
    print(link.motor_pos[1]%np.pi)
    p_constants = [0.0, 0.0]
    d_constants = [0.0, 0.0]
    link.set_pd(p_constants, d_constants)
