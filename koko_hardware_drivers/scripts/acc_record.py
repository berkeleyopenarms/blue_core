#!/usr/bin/env python
import time
import rospy
import math
from geometry_msgs.msg import Vector3
import numpy as np
from koko_hardware_drivers.msg import MotorState
import tf.transformations as transformations

CONTROL_LOOP_FREQ = 100

global x_accum
global y_accum
global z_accum
global num

exp_const = 0.99
x_accum = 0.0
y_accum = 0.0
z_accum = 0.0
num = 0

def rotation_matrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])
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
        x_accum = x_accum * exp_const + x_acc * (1.0 - exp_const)
        y_accum = y_accum * exp_const + y_acc * (1.0 - exp_const)
        z_accum = z_accum * exp_const + z_acc * (1.0 - exp_const)



#################################################################################################

def main():
    global x_accum
    global y_accum
    global z_accum
    global num

    rospy.init_node('acc_recorder', anonymous=True)
    rospy.Subscriber("/koko_hardware/motor_states", MotorState, get_accel, queue_size=1)
    grav_pub = rospy.Publisher("/koko_hardware/gravity", Vector3, queue_size=1)
    r = rospy.Rate(CONTROL_LOOP_FREQ)

    while not rospy.is_shutdown():
        if num%100 == 0:
            axis = [0.0, 0.0, 1.0]
            # correction z rotation
            theta = 4.301 - np.pi
            best_z = np.linalg.inv(rotation_matrix(axis, theta))
            best_z = transformations.rotation_matrix(-theta, axis)[:3,:3]

            print 'iteration {}'.format(num)
            print 'x_accum: {}'.format(x_accum)
            print 'y_accum: {}'.format(y_accum)
            print 'z_accum: {}'.format(z_accum)
            raw = np.array([[x_accum],[y_accum],[z_accum]])
            corrected = best_z.dot(raw)
            norm_val = np.linalg.norm(corrected) / 9.81
            grav_msg = Vector3()
            grav_msg.x = -corrected[0] / norm_val
            grav_msg.y = -corrected[1] / norm_val
            grav_msg.z = -corrected[2] / norm_val
            grav_pub.publish(grav_msg)


        r.sleep()

if __name__ == '__main__':
    main()
