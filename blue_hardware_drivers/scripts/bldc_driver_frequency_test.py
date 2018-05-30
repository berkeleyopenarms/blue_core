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

    s = serial.Serial(port=port, baudrate=1000000, timeout=0.01)
    device = BLDCControllerClient(s)
    time.sleep(0.1)

    device.leaveBootloader(motor_ids)

    time.sleep(0.2)
    s.flush()
    time.sleep(0.1)


    freq_pub = rospy.Publisher("/freq", Float32, queue_size=1)
    msg = Float32()

    last_time = rospy.get_time()

    for id in motor_ids:
        success = False
        for attempt in range(5):
            try:
                rospy.loginfo("Calibrating motor %d..." % id)
                #calibrations = device.readCalibration([id])
                #print(calibrations)
                #device.setZeroAngle([id], [calibrations['angle']])
                #device.setInvertPhases([id], [calibrations['inv']])
                #device.setERevsPerMRev([id], [calibrations['epm']])
                #device.setTorqueConstant([id], [calibrations['torque']])
                #device.setPositionOffset([id], [calibrations['zero']])
                device._ser.read_all()
                device.setZeroAngle([id], [1169])
                device.setInvertPhases([id], [1])
                device.setERevsPerMRev([id], [14])
                device.setTorqueConstant([id], [1.45])
                device.setPositionOffset([id], [0.0])
                device.setCurrentControlMode([id])
                #starting_angles[id] = 0.0
                device.writeRegisters([id], [0x1030], [1], [struct.pack('<H', 1000)])
                rospy.loginfo("Motor %d ready: supply voltage=%fV", id, device.getVoltage([id])[0])
                success = True
                break
            except Exception as e:
                rospy.logwarn(str(e))
                rospy.sleep(0.2)
        if not success:
            rospy.logerr("Could not calibrate motors")
            rospy.signal_shutdown("Could not calibrate motors")
            exit()

    while not rospy.is_shutdown():

        for i in range(10): # "low pass filter"
            try:
                # curr_angle = device.getRotorPosition(motor_id)
                state = device.setCommandAndGetState(motor_ids, [1.0]*len(motor_ids))
                # state = device.setCommand(motor_id, 0)
                # state = device.getState(motor_id)
            except Exception as e:
                rospy.logerr(str(e))

        current_time = rospy.get_time()
        dt = current_time - last_time
        freq = 1.0 / dt * 10
        msg.data = freq
        freq_pub.publish(msg)
        last_time = current_time

        rospy.loginfo_throttle(0.5, "{} / {}".format(freq, freq * len(motor_ids)))

if __name__ == '__main__':
    main()
