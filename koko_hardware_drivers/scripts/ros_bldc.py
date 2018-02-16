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
from geometry_msgs.msg import Vector3
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

    global device
    global command_queue
    global stop_motors

    # Set up a signal handler so Ctrl-C causes a clean exit
    def sigintHandler(signal, frame):
        print 'quitting'
        sys.exit()
    signal.signal(signal.SIGINT, sigintHandler)

    # Find and connect to the motor controller
    port = sys.argv[1]
    s = serial.Serial(port=port, baudrate=1000000, timeout=0.1)
    device = BLDCControllerClient(s)

    # Initial hardware setup
    for id in mapping:
        rospy.loginfo("Booting motor %d..." % id)
        device.leaveBootloader(id)
        rospy.sleep(0.1)
        s.flush()
    s.flush()

    calibration_success = False
    for attempt in range(5):
        try:
            # Write calibration values
            for id in mapping:
                rospy.loginfo("Calibrating motor %d..." % id)
                calibrations = device.readCalibration(id)
                device.setZeroAngle(id, calibrations['angle'])
                device.setCurrentControlMode(id)
                device.setInvertPhases(id, calibrations['inv'])
                device.setERevsPerMRev(id, calibrations['epm'])
                device.writeRegisters(id, 0x1022, 1, struct.pack('<f', calibrations['torque']))
            calibration_success = True
            break
        except Exception as e:
            rospy.logwarn(str(e))
            rospy.sleep(0.5)
            pass

    if not calibration_success:
        rospy.logerr("Could not calibrate motors")
        rospy.signal_shutdown("Could not calibrate motors")
        exit()

    rospy.loginfo("Successfully calibrated motors")

    state_publisher = rospy.Publisher("motor_states", MotorState, queue_size=1)
    for key in mapping:
        rospy.Subscriber(mapping[key] + "_cmd", Float64, makeSetCommand(key), queue_size=1)

    # subscriber listens for a stop motor command to set zero current to motors if something goes wrong
    rospy.Subscriber("stop_motors", Bool, set_motor_current_zero, queue_size=1)

    angle_start = {}
    for id in mapping:
        angle_start[id] = device.getRotorPosition(id)
        rospy.loginfo("Motor %d startup: supply voltage=%fV", id, device.getVoltage(id))

    r = rospy.Rate(CONTROL_LOOP_FREQ)
    while not rospy.is_shutdown():
        stateMsg = MotorState()
        for key in mapping:
            try:
                if not stop_motors:
                    if key in command_queue:
                        state = device.setCommandAndGetState(key, command_queue.pop(key))
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

                grav_vect = Vector3()
                grav_vect.x = accel_x
                grav_vect.y = accel_y
                grav_vect.z = accel_z
                stateMsg.gravity_vector.append(grav_vect)

                if temperature > MAX_TEMP_WARN:
                    rospy.logwarn_throttle(1, "Motor " + str(key) +  " is overheating, currently at  " + str(temperature) + " degrees C")
                    if temperature > MAX_TEMP_MOTORS_OFF:
                        stop_motors = True
                        rospy.logerr("Motor " + str(key) +  " is too hot, setting motor currents to zero" )

            except Exception as e:
                rospy.logerr("Motor " + str(key) +  " driver error: " + str(e))

        state_publisher.publish(stateMsg)
        r.sleep()

if __name__ == '__main__':
    main()
