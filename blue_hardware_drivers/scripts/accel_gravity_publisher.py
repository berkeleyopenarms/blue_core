#!/usr/bin/env python

### TODO: to be moved into hardware interface

import rospy
from geometry_msgs.msg import Vector3
from blue_hardware_drivers.msg import MotorState
import numpy as np
import tf.transformations as transformations

UPDATE_FREQ = 200
EXP_CONST = 0.99

global x_accum
global y_accum
global z_accum
global num

x_accum = 0.0
y_accum = 0.0
z_accum = 9.8
num = 0

def get_accel(msg):
    global x_accum
    global y_accum
    global z_accum
    global num
    base_name = 'base_roll_motor'
    index = -1
    for i, name_test in enumerate(msg.name):
        if base_name == name_test:
            index = i

    acc_vect = msg.accel[index]
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
        x_accum = x_accum * EXP_CONST + x_acc * (1.0 - EXP_CONST)
        y_accum = y_accum * EXP_CONST + y_acc * (1.0 - EXP_CONST)
        z_accum = z_accum * EXP_CONST + z_acc * (1.0 - EXP_CONST)

#################################################################################################

def main():
    global x_accum
    global y_accum
    global z_accum
    global num

    rospy.init_node("gravity_publisher", anonymous=True)
    rospy.Subscriber("blue_hardware/motor_states", MotorState, get_accel, queue_size=1)
    grav_pub = rospy.Publisher("blue_hardware/gravity", Vector3, queue_size=1)
    r = rospy.Rate(UPDATE_FREQ)

    while not rospy.is_shutdown():
        if num % 10 == 0:
            axis = [0.0, 0.0, 1.0]

            # correction for z rotation
            theta = 4.301 - np.pi
            best_z = transformations.rotation_matrix(-theta, axis)[:3,:3]

            #print 'iteration {}'.format(num)
            #print 'x_accum: {}'.format(x_accum)
            #print 'y_accum: {}'.format(y_accum)
            #print 'z_accum: {}'.format(z_accum)
            raw = np.array([[x_accum],[y_accum],[z_accum]])
            corrected = best_z.dot(raw)
            norm_val = np.linalg.norm(corrected) / 9.81
            grav_msg = Vector3()
            grav_msg.x = corrected[0] / norm_val
            grav_msg.y = corrected[1] / norm_val
            grav_msg.z = -corrected[2] / norm_val
            grav_pub.publish(grav_msg)

        r.sleep()

if __name__ == '__main__':
    main()
