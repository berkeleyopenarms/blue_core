#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from koko_hardware_drivers.msg import MotorState
import numpy as np
import tf.transformations as transformations

UPDATE_FREQ = 20
EXP_CONST = 0.99

global x_accum
global y_accum
global z_accum
global num

x_accum = [0.0, 0.0]
y_accum = [0.0, 0.0]
z_accum = [0.0, 0.0]
num = [0,0]

# def rotation_matrix(axis, theta):
#     axis = np.asarray(axis)
#     axis = axis/math.sqrt(np.dot(axis, axis))
#     a = math.cos(theta/2.0)
#     b, c, d = -axis*math.sin(theta/2.0)
#     aa, bb, cc, dd = a*a, b*b, c*c, d*d
#     bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
#     return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
#                      [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
#                      [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def get_accel(msg):
    global x_accum
    global y_accum
    global z_accum
    global num
    right_name = 'right_motor'
    left_name = 'left_motor'
    index = -1
    loc = -1
    for i, name_test in enumerate(msg.name):
        if right_name == name_test:
            index = i
            loc = 0
        if left_name == name_test:
            index = i
            loc = 1
            # print(name_test)

        acc_vect = msg.accel[index]
        x_acc = acc_vect.x
        y_acc = acc_vect.y
        z_acc = acc_vect.z
        if num[loc] == 0:
            num[loc] += 1
            x_accum[loc] = x_acc
            y_accum[loc] = y_acc
            z_accum[loc] = z_acc
        else:
            num[loc] += 1
            x_accum[loc] = x_accum[loc] * EXP_CONST + x_acc * (1.0 - EXP_CONST)
            y_accum[loc] = y_accum[loc] * EXP_CONST + y_acc * (1.0 - EXP_CONST)
            z_accum[loc] = z_accum[loc] * EXP_CONST + z_acc * (1.0 - EXP_CONST)

#################################################################################################

def main():
    global x_accum
    global y_accum
    global z_accum
    global num

    rospy.init_node('acc_recorder', anonymous=True)
    rospy.Subscriber("koko_hardware/motor_states", MotorState, get_accel, queue_size=1)
    grav_pub0 = rospy.Publisher("koko_hardware/gravity0", Vector3, queue_size=1)
    grav_pub1 = rospy.Publisher("koko_hardware/gravity1", Vector3, queue_size=1)
    r = rospy.Rate(UPDATE_FREQ)

    while not rospy.is_shutdown():

        #rospy.logerr("{}".format(num))
        #rospy.logerr("{}".format(num))
        #axis = [0.0, 0.0, 1.0]
        # correction z rotation
        #theta = 4.301 - np.pi
        # best_z = np.linalg.inv(rotation_matrix(axis, theta))
        #best_z = transformations.rotation_matrix(-theta, axis)[:3,:3]

        #print 'iteration {}'.format(num)
        #print 'x_accum: {}'.format(x_accum)
        #print 'y_accum: {}'.format(y_accum)
        #print 'z_accum: {}'.format(z_accum)

        # right

        raw = np.array([[x_accum[0]],[y_accum[0]],[z_accum[0]]])
        grav_msg = Vector3()
        grav_msg.x = raw[0]
        grav_msg.y = raw[1]
        grav_msg.z = raw[2]
        grav_pub0.publish(grav_msg)

        # left
        axis = [0.0, 0.0, 1.0]
        theta = np.pi
        correction_transform = transformations.rotation_matrix(theta, axis)[:3,:3]
        raw = np.array([[x_accum[1]],[y_accum[1]],[z_accum[1]]])
        raw = correction_transform.dot(raw)

        axis = [1.0, 0.0, 0.0]
        theta = np.pi
        correction_transform = transformations.rotation_matrix(theta, axis)[:3,:3]
        raw = correction_transform.dot(raw)

        grav_msg = Vector3()
        grav_msg.x = raw[0]
        grav_msg.y = raw[1]
        grav_msg.z = raw[2]
        grav_pub1.publish(grav_msg)
        print("published")
        r.sleep()

if __name__ == '__main__':
    main()
