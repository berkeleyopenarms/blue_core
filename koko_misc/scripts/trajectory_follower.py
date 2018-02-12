#!/usr/bin/env python

import csv
import numpy
import time
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
global joint
def joint_callback(msg):
    joints = msg.position

def main():
    reader = csv.reader(open("trajectory_vert.csv", "rb"), delimiter=",")
    # reader = csv.reader(open("trajectory.csv", "rb"), delimiter=",")
    x = list(reader)
    traj = numpy.array(x).astype("float")

    rospy.init_node('pd_publisher', anonymous=True)

    joint_pub = rospy.Publisher("/koko_controllers/joint_position_controller/command", Float64MultiArray, queue_size=1)
    rospy.Subscriber( "joint_states", JointState, joint_callback, queue_size=1)

    while not rospy.is_shutdown():
        for col in traj.T:
            joint_msg = Float64MultiArray()

            joint_msg.data = col
            joint_pub.publish(joint_msg)
            print('New Joint Mesage published')
            rospy.sleep(0.1)

if __name__ == '__main__':
    main()
