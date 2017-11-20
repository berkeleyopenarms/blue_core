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

mapping = {3: "base_roll_motor"}
# mapping = {15: "base_roll_motor", 11: "right_motor1", 12: "left_motor1", 14: "right_motor2", \
#            16: "left_motor2", 21: "right_motor3", 19: "left_motor3"}

##################################################################################################

def main():
    rospy.init_node('freq_publisher', anonymous=True)

    port = sys.argv[1]
    print port
    s = serial.Serial(port=port, baudrate=1000000, timeout=0.001)
    print s.BAUDRATES
    device = BLDCControllerClient(s)
    time.sleep(0.1)
    for key in mapping:
        device.leaveBootloader(key)
        time.sleep(0.1)
        s.flush()
        time.sleep(0.1)
    time.sleep(0.1)

    last_time = rospy.get_time()

    freq_pub = rospy.Publisher("/freq", Float32, queue_size=1)
    msg = Float32()

    while not rospy.is_shutdown():
        for key in mapping:
            try:
                curr_angle = device.getEncoder(key)

            except Exception as e:
                rospy.logerr(str(e))
                rospy.logerr(str(key    ))

        current_time = rospy.get_time()
        dt = current_time - last_time
        freq = 1 / dt
        msg.data = freq
        freq_pub.publish(msg)
        last_time = current_time



if __name__ == '__main__':
    main()
