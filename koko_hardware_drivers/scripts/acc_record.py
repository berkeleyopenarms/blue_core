#!/usr/bin/env python
import time
import rospy
import math
from geometry_msgs.msg import Vector3
from koko_hardware_drivers.msg import MotorState

CONTROL_LOOP_FREQ = 100

global x_accum
global y_accum
global z_accum
global num

x_accum = 0.0
y_accum = 0.0
z_accum = 0.0
num = 0

def get_accel(msg):
    global x_accum
    global y_accum
    global z_accum
    global num
    acc_vect = msg.accel[0]
    x_acc = acc_vect.x
    y_acc = acc_vect.y
    z_acc = acc_vect.z
    if num == 0:
        num += 1
        x_accum = x_acc
        y_accum = y_acc
        z_accum = z_acc
    else:
        num += 1
        ratio = 1.0 / float(num)
        x_accum = x_accum * (1.0 - ratio) + x_acc * ratio
        y_accum = y_accum * (1.0 - ratio) + y_acc * ratio
        z_accum = z_accum * (1.0 - ratio) + z_acc * ratio



#################################################################################################

def main():
    global x_accum
    global y_accum
    global z_accum
    global num

    rospy.init_node('acc_recorder', anonymous=True)
    rospy.Subscriber("/koko_hardware/motor_states", MotorState, get_accel, queue_size=1)
    r = rospy.Rate(CONTROL_LOOP_FREQ)

    while not rospy.is_shutdown():
        if num%100 == 0:
            print 'iteration {}'.format(num)
            print 'x_accum: {}'.format(x_accum)
            print 'y_accum: {}'.format(y_accum)
            print 'z_accum: {}'.format(z_accum)
        r.sleep()

if __name__ == '__main__':
    main()
