#!/usr/bin/env python
from blue_hardware_drivers import PyBLDCControllerClient
import sys
import rospy

# Example usage:
#    rosrun blue_hardware_drivers bldc_driver_frequency_test.py 2,42,41,33,16,31,32,14 /dev/ttyUSB1

ENCODER_ANGLE_PERIOD = 1 << 14
MAX_CURRENT = 1.2
CONTROL_LOOP_FREQ = 500

port_default = "/dev/ttyUSB0"

##################################################################################################

def main():
    rospy.init_node('freq_publisher', anonymous=True)
    motor_ids = [int(x) for x in sys.argv[1].split(",")]
    if len(sys.argv) >= 3:
        port = sys.argv[2]
    else:
        port = port_default

    rospy.loginfo("Testing communication frequency: {} {}".format(motor_ids, port))

    device = PyBLDCControllerClient(port, motor_ids)
    device.reset_boards()
    device.queue_get_rotor_position(motor_ids)
    device.exchange()
    rospy.logerr("result: {}".format(motor_ids, port))


if __name__ == '__main__':
    main()
