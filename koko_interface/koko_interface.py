#!/usr/bin/env python
from rosbridge_client import ROSBridgeClient
import csv
import numpy
import time


class KokoInterface:
    """ a class to control a Koko"""

    def __init__(self, ip, port=9090, buffer_size=4096):
        self.RBC = ROSBridgeClient('hekate.cs.berkeley.edu')
        self.ip = ip
        self.port = port
        self.buffer_size = buffer_size
        _ROS_SET_JOINT_ANGLES_TOPIC = '/koko_controllers/joint_position_controller/command'
        _ROS_GET_JOINT_ANGLES_TOPIC = '/joint_states'
        print("Testing KokoInterface")
        self._joint_angles_subscriber = self.RBC.subscriber(_ROS_GET_JOINT_ANGLES_TOPIC, "sensor_msgs/JointState", self._joint_angles_callback)
        self._joint_angles_publisher = self.RBC.publisher(_ROS_SET_JOINT_ANGLES_TOPIC, "std_msgs/Float64MultiArray")
        self.is_debug = True

    def _joint_angles_callback(self, data):
        self.joint_angles = data
        if self.is_debug:
            print(self.joint_angles)

    def set_joint_angle_target(self, joint_angles):
        """ where joint_positions is a list of 7 angles in radians"""
        joint_angles_message = {
            "layout" : {},
            "data": joint_angles
        }
        while True:
            self._joint_angles_publisher.publish(joint_angles_message)

    def get_joint_angles(self):
        return self.joint_angles

if __name__== '__main__':
    koko = KokoInterface('hekate.cs.berkeley.edu')
    koko.set_joint_angle_target([0,0,0,0,0,0,0])
    koko.get_joint_angles()
