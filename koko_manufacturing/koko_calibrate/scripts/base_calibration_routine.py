#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from koko_hardware_drivers.msg import MotorState

class Base:
    def __init__(self):
        self.offset = 0.52935836
        p_constants = [58.0]
        d_constants = [1.0]

        rospy.init_node('Base_Calibration', anonymous=True)

        rate = rospy.Rate(200)

        self.p_publisher = rospy.Publisher("/p_terms", Float64MultiArray, queue_size=1)
        self.d_publisher = rospy.Publisher("/d_terms", Float64MultiArray, queue_size=1)

        self.cmd = rospy.Publisher("/koko_controllers/joint_positions_controller/command", Float64MultiArray, queue_size=1)
        self.pos = [0.0]
        self.motor_pos = [0.0]

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
        # print(cmds)
        j_cmd = Float64MultiArray()
        j_cmd.data = cmds
        self.cmd.publish(j_cmd)

    def update_motors(self, joint_msg):
        self.motor_pos[0] = joint_msg.position[0]

    def update_joints(self, joint_msg):
        self.pos[0] = joint_msg.position[0]

    def center_joint(self, j=0, _iter=1):
        max_pos_list = []
        min_pos_list = []
        for _ in range(_iter):
            min_pos_list.append(self.find_end(j, -1.0))
            max_pos_list.append(self.find_end(j, 1.0))
        print(max_pos_list)
        print(min_pos_list)
        max_pos = sum(max_pos_list) / float(_iter)
        min_pos = sum(min_pos_list) / float(_iter)
        return (max_pos + min_pos) / 2.0, max_pos, min_pos

    def find_end(self, j, direction):
        eps = 2.5 # joint position erro in radians
        curr_error = 0
        delta = 0.06
        command_pos = self.pos[:]
        while curr_error < eps and not rospy.is_shutdown():
            # rospy.logerr("{}".format(curr_error))
            command_pos[j] = command_pos[j] + direction *  delta
            self.set_pos(command_pos)
            curr_error = np.abs(command_pos[j] - self.pos[j])
            # rospy.logerr("{}".format(curr_error))
            rospy.sleep(0.03)

        rospy.sleep(0.5)
        rospy.loginfo('done finding the end')
        return self.pos[j]

    def find_pos(self, pos_targ):
        j = 0
        eps = 2.5 # joint position erro in radians
        curr_error = 0
        delta = 0.06
        command_pos = self.pos[:]
        direction = 1.0
        print('Finding end')
        if pos_targ > self.pos[0]:
            direction = -1.0
        while np.abs(self.pos[0] - pos_targ) > eps and not rospy.is_shutdown():
            # rospy.logerr("{}".format(curr_error))
            command_pos[j] = command_pos[j] + direction *  delta
            self.set_pos(command_pos)
            # print('Finding end')
            curr_error = np.abs(command_pos[j] - self.pos[j])
            # rospy.logerr("{}".format(curr_error))
            rospy.sleep(0.02)

        rospy.sleep(0.5)
        rospy.loginfo('done finding the end')
        return self.pos[j]
if __name__ == '__main__':
    link = Base()
    iterations = 5
    link = Base()
    rospy.loginfo('Link setup complete')
    # set to start up position
    rospy.sleep(2)
    rospy.loginfo('Set to zero')
    rospy.sleep(2)
    # centering roll link first
    center_roll, max_roll, min_roll = link.center_joint(_iter=iterations)
    print(max_roll - min_roll)
    go_to_value = center_roll - link.offset
    print('Finding end')
    link.find_pos(go_to_value)
    rospy.sleep(2)
    print(link.pos[0])
    print(go_to_value)
    print(link.pos[0])

    rospy.sleep(2)
    p_constants = [40.0]
    d_constants = [1.0]
    link.set_pd(p_constants, d_constants)
    rospy.sleep(2)
    link.set_pos([go_to_value])
    link.find_pos(go_to_value)
    print(center_roll, "center roll")
    rospy.sleep(2)
    rospy.loginfo('Script Complete')

    rospy.sleep(1)
    p_constants = [0.0]
    d_constants = [0.0]
    link.set_pd(p_constants, d_constants)

    print("Motor Position")
    print(link.motor_pos[0])
    print(link.motor_pos[0])
    print(link.motor_pos[0]%(2*np.pi))
