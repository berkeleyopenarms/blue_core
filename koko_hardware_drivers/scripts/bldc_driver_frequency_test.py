#!/usr/bin/env python
from comms import BLDCControllerClient
import time
import serial
import math
import signal
import sys
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from comms import *


ENCODER_ANGLE_PERIOD = 1 << 14
MAX_CURRENT = 1.2
CONTROL_LOOP_FREQ = 500

port_default = "/dev/ttyUSB0"

##################################################################################################

def main():
    rospy.init_node('freq_publisher', anonymous=True)

    motor_id = int(sys.argv[1])

    if len(sys.argv) >= 3:
        port = sys.argv[2]
    else:
        port = port_default

    rospy.loginfo("Testing communication frequency: {} {}".format(motor_id, port))

    s = serial.Serial(port=port, baudrate=1000000, timeout=0.1)
    device = BLDCControllerClient(s)
    time.sleep(0.1)

    device.leaveBootloader(motor_id)
    time.sleep(0.1)
    s.flush()
    time.sleep(0.1)

    last_time = rospy.get_time()

    freq_pub = rospy.Publisher("/freq", Float32, queue_size=1)
    msg = Float32()

    while not rospy.is_shutdown():

        for i in range(10): # "low pass filter"
            try:
                curr_angle = device.getRotorPosition(motor_id)
            except Exception as e:
                rospy.logerr(str(e))
                rospy.logerr(str(motor_id))

        current_time = rospy.get_time()
        dt = current_time - last_time
        freq = 1.0 / dt * 10
        msg.data = freq
        freq_pub.publish(msg)
        last_time = current_time

        rospy.loginfo_throttle(0.5, freq)

if __name__ == '__main__':
    main()
