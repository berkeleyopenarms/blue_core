#!/usr/bin/env python
from comms import BLDCControllerClient
import time
import serial
import math
import signal
import sys
import rospy
import json
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from koko_hardware_drivers.msg import MotorState
from comms import *


ENCODER_ANGLE_PERIOD = 1 << 14
MAX_CURRENT = 1.5
MAX_TEMP_WARN = 55 # degrees C
MAX_TEMP_MOTORS_OFF = 65
CONTROL_LOOP_FREQ = 1000

#name_mapping = {15: "base_roll_motor", 14: "right_motor1", 16: "left_motor1", \
#           10: "right_motor2", 17: "left_motor2", 20: "right_motor3", 18: "left_motor3" }#, 2: "gripper_motor"}
#angle_mapping = {15: 13002, 14: 4484, 16: 2373, 10: 11067, 17: 10720, 20: 3839, 18: 284 }#, 2: 11349}
#erevs_per_mrev_mapping = {15: 14, 14: 14, 16: 14, 10: 14, 17: 14, 20: 21, 18: 21 }#, 2: 21}
#invert_mapping = {15: False, 14: False, 16: False, 10: False, 17: True, 20: False, 18: False }#, 2: False}
#torque_constant_mapping = {15: 1.45, 14: 1.45, 16: 1.45, 10: 1.45, 17: 1.45, 20: 0.6 , 18: 0.6 }#, 2: False}

mapping = {15: "base_roll_motor", 14: "right_motor1", 16: "left_motor1", \
           10: "right_motor2", 17: "left_motor2", 20: "right_motor3", 18: "left_motor3" }#, 2: "gripper_motor"}
#mapping = {15: "base_roll_motor", 10: "right_motor1", 17: "left_motor1", \
#          14: "right_motor2", 16: "left_motor2", 20: "right_motor3", 18: "left_motor3" }#, 2: "gripper_motor"}
# name_mapping = {15: "base_roll_motor", 14: "right_motor1", 16: "left_motor1", }
# angle_mapping = {15: 13002, 14: 4484, 16: 2373}
# erevs_per_mrev_mapping = {15: 14, 14: 14, 16: 14}
# invert_mapping = {15: False, 14: False, 16: False}
# torque_constant_mapping = {15: 1.45, 14: 1.45, 16: 1.45}
#
# name_mapping = {15: "base_roll_motor", 14: "right_motor1"}
# angle_mapping = {15: 13002, 14: 4484}
# erevs_per_mrev_mapping = {15: 14, 14: 14}
# invert_mapping = {15: False, 14: False}
# torque_constant_mapping = {15: 1.45, 14: 1.45}
#
# name_mapping = {15: "base_roll_motor"}
# angle_mapping = {15: 13002}
# erevs_per_mrev_mapping = {15: 14}
# invert_mapping = {15: False}
# torque_constant_mapping = {15: 1.45}

# flip_mapping = {21: True}
flip_mapping = {}


# name_mapping = {15: "gripper_motor"}
# angle_mapping = {15: 13002}
# erevs_per_mrev_mapping = {15: 14}
# invert_mapping = {15: False}

global command_queue
command_queue = {}
global stop_motors
stop_motors = False
#################################################################################################
############### Helper Functions ################################################################
#################################################################################################

def setpointFunction(t):
    return math.sin(t * 2 * math.pi * 0.5) * 5.0

def makeSetCommand(key):
    def setCommand(key, msg):
        global device
        global command_queue
        effort_raw = msg.data
        effort_filtered = effort_raw

        if key in flip_mapping and flip_mapping[key]:
            effort_filtered = -effort_filtered

        if effort_filtered > MAX_CURRENT:
            effort_filtered = MAX_CURRENT
        elif effort_filtered < -MAX_CURRENT:
            effort_filtered = -MAX_CURRENT
        command_queue[key] = effort_filtered
    return lambda msg: setCommand(key, msg)

def set_motor_current_zero(msg):
    global stop_motors
    if msg.data:
        stop_motors = True
    else:
        stop_motors = False
#################################################################################################

def main():
    rospy.init_node('jointInterface', anonymous=True)
    #rate = rospy.Rate(CONTROL_LOOP_FREQ)
    global device
    global command_queue
    global stop_motors

    # Find and connect to the motor controller
    port = sys.argv[1]
    s = serial.Serial(port=port, baudrate=1000000, timeout=0.1)
    device = BLDCControllerClient(s)
    for key in mapping:
        device.leaveBootloader(key)
        s.flush()
    time.sleep(0.5)

    state_publisher = rospy.Publisher("motor_states", MotorState, queue_size=1)
    for key in mapping:
        rospy.Subscriber(mapping[key] + "_cmd", Float64, makeSetCommand(key), queue_size=1)

    # subscriber listens for a stop motor command to set zero current to motors if something goes wrong
    rospy.Subscriber("stop_motors", Bool, set_motor_current_zero, queue_size=1)
    # Set up a signal handler so Ctrl-C causes a clean exit
    def sigintHandler(signal, frame):
        print 'quitting'
        sys.exit()
    signal.signal(signal.SIGINT, sigintHandler)

    time.sleep(0.2)

    # Write calibration values
    for id in mapping:
        rospy.loginfo("Calibrating motor %d" % id)
        calibrations = device.readCalibration(id)
        device.setZeroAngle(id, calibrations['angle'])
        device.setCurrentControlMode(id)
        device.setInvertPhases(id, calibrations['inv'])
        device.setERevsPerMRev(id, calibrations['epm'])
        device.writeRegisters(id, 0x1022, 1, struct.pack('<f', calibrations['torque']))

    ## Old calibration method:
    #for id, angle in angle_mapping.items():
    #    rospy.loginfo("Calibrating motor %d" % id)
    #    device.setZeroAngle(id, angle)
    #    device.setCurrentControlMode(id)

    #for id, invert in invert_mapping.items():
    #    device.setInvertPhases(id, int(invert))

    #for id, erevs_per_mrev in erevs_per_mrev_mapping.items():
    #    device.setERevsPerMRev(id, int(erevs_per_mrev))

    #for id, tc in torque_constant_mapping.items():
    #    device.writeRegisters(id, 0x1022, 1, struct.pack('<f', tc))

    start_time = time.time()
    time_previous = start_time
    angle_start = {}

    for id in mapping:
        angle_start[id] = device.getRotorPosition(id)
        rospy.loginfo("Motor %d startup: supply voltage=%fV", id, device.getVoltage(id))

    r = rospy.Rate(CONTROL_LOOP_FREQ)
    while not rospy.is_shutdown():
        stateMsg = MotorState()
        for key in mapping:
            try:
                curr_time = time.time()
                if not stop_motors:
                    if key in command_queue:
                        state = device.setCommandAndGetState(key, command_queue[key])
                    else:
                        state = device.getState(key)
                else:
                    state = device.setCommandAndGetState(key, 0.0)

                angle, velocity, direct_current, quadrature_current, \
                        supply_voltage, temperature, accel_x, accel_y, accel_z = state
                angle -= angle_start[key]

                stateMsg.name.append(mapping[key])
                stateMsg.position.append(angle)
                stateMsg.velocity.append(velocity)
                stateMsg.direct_current.append(direct_current)
                stateMsg.quadrature_current.append(quadrature_current)
                stateMsg.supply_voltage.append(supply_voltage)
                stateMsg.temperature.append(temperature)
                if temperature > MAX_TEMP_WARN:
                    rospy.logerr("Motor " + str(key) +  " is too hot, currently at  " + str(temperature) + " degrees C")
                    rospy.logerr("Please Shut of the motors")
                    if temperature > MAX_TEMP_MOTORS_OFF:
                        stop_motors = True
                        rospy.logerr("Motor " + str(key) +  " is too hot, setting motor currents to zero" )

            except Exception as e:
                rospy.logerr("Motor " + str(key) +  " driver error: " + str(e))

        state_publisher.publish(stateMsg)
        r.sleep()

if __name__ == '__main__':
    main()
