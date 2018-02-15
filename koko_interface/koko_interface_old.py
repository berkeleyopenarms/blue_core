#!/usr/bin/env python
import json
import time
import pprint
# import math3d as m3d
import numpy as np
from rosbridge_client import ROSBridgeClient

class KokoInterface:
    """ a class to control a Koko"""

    """ define rostopic-name variables"""
    # _ROS_GET_JOINT_ANGLES_TOPIC =
    # _ROS_SET_END_EFFECTOR_POSE_TOPIC =
    # _ROS_GET_END_EFFECTOR_POSE_TOPIC =
    # _ROS_SET_JOINT_TORQUE_TOPIC =
    # _ROS_GET_JOINT_TORQUE_TOPIC =
    # _ROS_GET_MOTOR_TEMP_TOPIC =

    def __init__(self, ip='127.0.0.1', port=9090, buffer_size=4096):
        self.RBC = ROSBridgeClient('127.0.0.1')
        self.ip = ip
        self.port = port
        self.buffer_size = buffer_size
        _ROS_SET_JOINT_ANGLES_TOPIC = '/koko_joint_angles'
        print("Testing KokoInterface")

        """ create relevant publishers"""
        self._joint_angles_publisher = self.RBC.publisher(_ROS_SET_JOINT_ANGLES_TOPIC, "std_msgs/Float64MultiArray")
        # self._end_effector_pose_publisher = self.RBC.publisher(_ROS_SET_END_EFFECTOR_POSE_TOPIC, "geometry_msgs/PoseStamped")
        # self._joint_torque_publisher = self.RBC.publisher(_ROS_SET_JOINT_TORQUE_TOPIC, "std_msgs/Float64MultiArray")

        # """ create relevant subscribers"""
        # self._joint_angles_subscriber = self.RBC.subscriber(_ROS_GET_JOINT_ANGLES_TOPIC, "std_msgs/Float64MultiArray", self._joint_angles_callback())
        # self._end_effector_pose_subscriber = self.RBC.subscriber(_ROS_GET_END_EFFECTOR_POSE_TOPIC, "geometry_msgs/PoseStamped", self._end_effector_pose_callback())


    # """ create callback functions for all subscribers I want?"""
    # def _joint_angles_callback(self, data):

    # def _end_effector_pose_callback(self, data):

    def set_joint_angle_target(self, joint_angles):
        """ where joint_positions is a list of 7 angles in radians"""
        joint_angles_message = {
            "layout" : {},
            "data": joint_angles
        }
        self._joint_angles_publisher.publish(joint_angles_message)


    # def get_joint_angles(self):
        # return self.joint_angles

    # def set_tcp(self, tcp):
        # """ set robot flange to tool tip transformation"""
        # if isinstance(tcp, m3d.Transform):
            # tcp = tcp.pose_vector


    # def set_csys(self, tranform):
        # """ set reference coordinate system to use """
        # self.csys = transform
#
    # def set_orientation(self, orient, acc=0.01, vel=0.01, wait=True, threshold=None):
        # """ set tool orientation using an orientation matrix from math3d or an orientatio vector"""
        # if not isinstance(orient, m3d.Orientation):
            # orient = m3d.Orientation(orient)
        # trans = self.get_pose()
if __name__== '__main__':
    while True:
        KokoInterface().set_joint_angle_target([1,2,3,4,5,6,7])
