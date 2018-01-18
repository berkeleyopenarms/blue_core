#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float32
import math

# proof of concept; this should be rewritten

class koko_gripper:
    def __init__(self):
        self.Kp = 0.1 # this is too low right now
        self.min_position = None
        self.max_position = None
        self.current_position = None
        self.position_setpoint = None

        rospy.init_node('koko_gripper')
        rospy.Subscriber('/right_trigger', Float32, self.trigger_callback)
        rospy.Subscriber('/DOF/gripper_State', JointState, self.state_callback)
        pub = rospy.Publisher('/DOF/gripper_Cmd', Float64, queue_size=2)
        rate = rospy.Rate(60) # 10hz

        while not rospy.is_shutdown():
            if self.current_position != None and self.position_setpoint != None:
                cmd = Float64()
                cmd.data = (self.position_setpoint - self.current_position) * self.Kp
                pub.publish(cmd)
            else:
                cmd = Float64()
                cmd.data = 0
                pub.publish(cmd)
            rate.sleep()

    def trigger_callback(self, value):
        x = value.data
        self.position_setpoint = (self.max_position - self.min_position) * x + self.min_position

    def state_callback(self, state):
        x = state.position[0]
        self.current_position = x

        if self.min_position == None:
            self.min_position = x
            self.max_position = x + math.pi * 1.3 * 2
            self.position_setpoint = x

if __name__ == '__main__':
    try:
        koko_gripper()
    except rospy.ROSInterruptException:
        pass
