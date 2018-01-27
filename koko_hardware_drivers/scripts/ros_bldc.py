#!/usr/bin/env python
from comms import BLDCControllerClient
import time
import serial
import math
import collections
import signal
import sys
import rospy
import json
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from comms import *


ENCODER_ANGLE_PERIOD = 1 << 14
MAX_CURRENT = 1.2
CONTROL_LOOP_FREQ = 1000
#mapping = {3: "right_motor2", 4: "left_motor2"} # mapping of id to joints
# mapping = {1: "left_motor", 2: "right_motor", 3: "right_motor2", 4: "left_motor2"} # mapping of id to joints
# angle_mapping = {1: 10356, 2: 13430, 3: 12164, 4: 8132} # mapping of id to joints
#angle_mapping = {3: 12164, 4: 8132} # mapping of id to joints


#mapping = {22: "base_roll_motor"} # mapping of id to joints
#mapping = {12: "left_motor1", 11: "right_motor1", 15: "base_roll_motor"} # mapping of id to joints
#mapping = {15: "base_roll_motor", 11: "right_motor1", 12: "left_motor1", 14: "right_motor2", \
           #16: "left_motor2", 21: "right_motor3", 19: "left_motor3"} # mapping of id to joints
#mapping = {15: "base_roll_motor", 11: "right_motor1", 12: "left_motor1", 10: "right_motor2", \
           #17: "left_motor2", 21: "right_motor3", 19: "left_motor3"} # mapping of id to joints
#mapping = {15: "base_roll_motor", 11: "right_motor1", 12: "left_motor1", 10: "right_motor2", \
           #17: "left_motor2", 21: "right_motor3", 19: "left_motor3", 2: "gripper_motor"} # mapping of id to joints
mapping = {15: "base_roll_motor", 14: "right_motor1", 16: "left_motor1", \
           10: "right_motor2", 17: "left_motor2", 21: "right_motor3", 19: "left_motor3" }#, 2: "gripper_motor"}
#angle_mapping = {22: 13002} # mapping of id to joints
#angle_mapping = {12: 1200, 11: 2164, 15: 13002} # mapping of id to joints
#angle_mapping = {15: 13002, 11: 2164, 12: 1200, 14: 4484, \
           #16: 2373, 21: 5899, 19: 2668}
#angle_mapping = {15: 13002, 11: 2164, 12: 1200,  \
           #17: 10720, 10: 11067, 21: 5899, 19: 2668}
#angle_mapping = {15: 13002, 11: 2164, 12: 1200,  \
                 #17: 10720, 10: 11067, 21: 5899, 19: 2668, 2: 11349}
angle_mapping = {15: 13002, 14: 4484, 16: 2373, 10: 11067, 17: 10720, 21: 5899, 19: 2668 }#, 2: 11349}
#erevs_per_mrev_mapping = {22: 14}
#erevs_per_mrev_mapping = {12: 14, 11: 14, 15: 14}
#erevs_per_mrev_mapping = {15: 14, 11: 14, 12: 14, 14: 14, 16: 14, 21: 21, 19: 21}
#erevs_per_mrev_mapping = {15: 14, 11: 14, 12: 14, 10: 14, 17: 14, 21: 21, 19: 21}
#erevs_per_mrev_mapping = {15: 14, 11: 14, 12: 14, 10: 14, 17: 14, 21: 21, 19: 21, 2: 21}
erevs_per_mrev_mapping = {15: 14, 14: 14, 16: 14, 10: 14, 17: 14, 21: 21, 19: 21 }#, 2: 21}
#invert_mapping = {22: False}
#invert_mapping = {12: False, 11: False, 15: False}
#invert_mapping = {15: False, 11: False, 12: False, 14: False, 16: False, 21: False, 19: False}
#invert_mapping = {15: False, 11: False, 12: False, 10: False, 17: True, 21: False, 19: False}
#invert_mapping = {15: False, 11: False, 12: False, 10: False, 17: True, 21: False, 19: False, 2: False}
invert_mapping = {15: False, 14: False, 16: False, 10: False, 17: True, 21: False, 19: False }#, 2: False}

flip_mapping = {21: True}


# invert_mapping = {2: False}
# erevs_per_mrev_mapping = {2: 21}
# angle_mapping = {2: 11349}
# mapping = {2: "gripper_motor"} # mapping of id to joints


# mapping = {15: "base_roll_motor", 11: "right_motor1", 12: "left_motor1", 14: "right_motor2", 16: "left_motor2", 21: "right_motor3", 19: "left_motor3" } # mapping of id to joints
# angle_mapping = {15: 13002, 11: 2164, 12: 1200, 14: 4484, 16: 2373, 21: 5899, 19: 2668} # mapping of id to joints
# erevs_per_mrev_mapping = {15: 14, 11: 14, 12: 14, 14: 14, 16: 14, 21: 14, 19: 14}
# invert_mapping = {15: 0, 11: 0, 12: 0, 14: 0, 16: 0, 21: 0, 19: 0}


global command_queue
command_queue = {}
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
        print "I heard " + str(effort_filtered)
    return lambda msg: setCommand(key, msg)

#################################################################################################

def main():
    rospy.init_node('jointInterface', anonymous=True)
    #rate = rospy.Rate(CONTROL_LOOP_FREQ)
    global device
    global command_queue

    # Find and connect to the motor controller
    port = sys.argv[1]
    s = serial.Serial(port=port, baudrate=1000000, timeout=0.1)
    device = BLDCControllerClient(s)
    for key in mapping:
        device.leaveBootloader(key)
        s.flush()
    time.sleep(0.5)


    pubArray = {}
    pubCurrArray = {}
    subArray = {}
    for key in mapping:
        pubArray[key] = rospy.Publisher("/DOF/" + mapping[key] + "_State", JointState, queue_size=1)
        pubCurrArray[key] = rospy.Publisher("/DOF/" + mapping[key] + "_Current", Float32, queue_size=1)
        subArray[key] = rospy.Subscriber("/DOF/" + mapping[key] + "_Cmd", Float64, makeSetCommand(key), queue_size=1)

    # Set up a signal handler so Ctrl-C causes a clean exit

    def sigintHandler(signal, frame):
        print 'quitting'
        sys.exit()
    signal.signal(signal.SIGINT, sigintHandler)

    time.sleep(0.2)

    # # Write calibration values
    # for id in mapping:
    #     calibrations = device.readCalibration(id)
    #     device.setZeroAngle(id, calibrations['angle'])
    #     device.setCurrentControlMode(id)
    #     device.setInvertPhases(id, calibrations['inv'])
    #     device.setERevsPerMRev(id, calibrations['epm'])

    # Old calibration method:
    for id, angle in angle_mapping.items():
        rospy.loginfo("Calibrating motor %d" % id)
        device.setZeroAngle(id, angle)
        device.setCurrentControlMode(id)

    for id, invert in invert_mapping.items():
        device.setInvertPhases(id, int(invert))

    for id, erevs_per_mrev in erevs_per_mrev_mapping.items():
        device.setERevsPerMRev(id, int(erevs_per_mrev))

    start_time = time.time()
    time_previous = start_time
    angle_start = {}
    velocity_window_size =10 
    recorded_positions = {} # map of rolling window of (time, position)

    for id in mapping:
        angle_start[id] = device.getRotorPosition(id)
        recorded_positions[id] = collections.deque()

    r = rospy.Rate(CONTROL_LOOP_FREQ)
    while not rospy.is_shutdown():
        for key in mapping:
            # try:
            # Compute the desired setpoint for the current time
            jointName = mapping[key]

            curr_time = time.time()
            if key in command_queue:
                curr_position = device.setCommandAndGetRotorPosition(key, command_queue[key]) - angle_start[key]
            else:
                curr_position = device.getRotorPosition(key) - angle_start[key]

            curr_velocity = 0
            recorded_positions[key].append((curr_time, curr_position))
            if len(recorded_positions[key]) >= velocity_window_size:
                prev_time, prev_position = recorded_positions[key].popleft()
                curr_velocity = (curr_position - prev_position) / (curr_time - prev_time)

            jointMsg = JointState()
            jointMsg.name = [jointName]
            jointMsg.position = [curr_position]
            jointMsg.velocity = [curr_velocity]
            jointMsg.effort = [0.0]
            pubArray[key].publish(jointMsg)

            currMsg = Float32()
            currMsg.data = float(0)
            pubCurrArray[key].publish(currMsg)

            # except Exception as e:
            #     rospy.logerr(str(e) + " " + str(key))

        command_queue = {}
        r.sleep()

if __name__ == '__main__':
    main()
