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
#mapping = {3: "right_motor2", 4: "left_motor2"} # mapping of id to joints
# mapping = {1: "left_motor", 2: "right_motor", 3: "right_motor2", 4: "left_motor2"} # mapping of id to joints
# angle_mapping = {1: 10356, 2: 13430, 3: 12164, 4: 8132} # mapping of id to joints
#angle_mapping = {3: 12164, 4: 8132} # mapping of id to joints


#mapping = {15: "base_roll_motor"} # mapping of id to joints
mapping = {12: "left_motor", 11: "right_motor", 15: "base_roll_motor"} # mapping of id to joints
#angle_mapping = {15: 13002} # mapping of id to joints
angle_mapping = {12: 1200, 11: 2164, 15: 13002} # mapping of id to joints
erevs_per_mrev_mapping = {12: 14, 11: 14, 15: 14}
#invert_mapping = {15: False}
invert_mapping = {12: False, 11: False, 15: False}

global command_queue
command_queue = {}
#################################################################################################
############### Helper Functions ################################################################
#################################################################################################

def setpointFunction(t):
	return math.sin(t * 2 * math.pi * 0.5) * 5.0

def getEncoderAngleRadians(device, key):
	return float(device.getEncoder(key)) 

def makeSetCommand(key):
	def setCommand(key, msg):
		global device
		global command_queue
		effort_raw = msg.data
		effort_filtered = effort_raw

		if effort_filtered > MAX_CURRENT:
			effort_filtered = MAX_CURRENT
		elif effort_filtered < -MAX_CURRENT:
			effort_filtered = -MAX_CURRENT
		command_queue[key] = effort_filtered
		print "I heard " + str(effort_filtered)
	return lambda msg: setCommand(key, msg)

##################################################################################################

def main():
	rospy.init_node('jointInterface', anonymous=True)
	#rate = rospy.Rate(CONTROL_LOOP_FREQ)
	global device
	global command_queue
	# Find and connect to the stepper motor controller
	port = sys.argv[1]
	s = serial.Serial(port=port, baudrate=1000000, timeout=0.001)
	print s.BAUDRATES
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
		# device.setParameter('iq_s', 0)   ###############################################################--> must fix at some point
		# device.setControlEnabled(False) ################################################################--> must fix at some point
		print 'quitting'
		sys.exit()
	signal.signal(signal.SIGINT, sigintHandler)

	# Set current to zero, enable current control loop
	for key in mapping:
		# device.setParameter(key, 'id_s', 0)
		# device.setParameter(key, 'iq_s', 0)
		# device.setControlEnabled(key, 0)########################### change to 1 when you want to actually control
		pass

	#angle = 0.0   # Treat the starting position as zero
	#last_mod_angle = getEncoderAngleRadians(device)
	start_time = time.time()
	time_previous = start_time
	angle_start = {}
	time.sleep(0.2)

	for id, angle in angle_mapping.items():
		# device.setInitialAngle(id, angle)
		device.writeRegisters(id, 0x0101, 1, struct.pack('<H', angle))
		device.writeRegisters(id, 0x0102, 1, struct.pack('<B', 0))

	for id, invert in invert_mapping.items():
		# device.setInitialAngle(id, angle)
		device.writeRegisters(id, 0x0109, 1, struct.pack('<B', int(invert)))

	for id, erevs_per_mrev in erevs_per_mrev_mapping.items():
		# device.setInitialAngle(id, angle)
		device.writeRegisters(id, 0x010A, 1, struct.pack('<B', int(erevs_per_mrev)))

	for key in mapping:
		angle_start[key] = getEncoderAngleRadians(device, key)

        r = rospy.Rate(CONTROL_LOOP_FREQ)
	while not rospy.is_shutdown():
		for key in mapping:
			try:
				# Compute the desired setpoint for the current time
				jointName = mapping[key]

				loop_start_time = time.time()
				delta_time = loop_start_time - time_previous
				time_previous = loop_start_time
				curr_angle = getEncoderAngleRadians(device, key)
                                jointMsg = JointState()
				jointMsg.name = [jointName]
				jointMsg.position = [curr_angle - angle_start[key]]
				#jointMsg.position = [0.0]
				jointMsg.velocity = [0.0]#[device.getVelocity(key)]
				jointMsg.effort = [0.0] 
				pubArray[key].publish(jointMsg)
				#print("name: " + jointName + "  position: " + str(jointMsg.position))
				#time.sleep(max(0.0, loop_start_time + 1.0 / CONTROL_LOOP_FREQ - time.time()))

				#currMsg = Float32()
				#currMsg.data = float(0)
				#pubCurrArray[key].publish(currMsg)
				if key in command_queue:
					device.setDuty(key, command_queue[key])
					pass
			except Exception as e:
				rospy.logerr(str(e))
				rospy.logerr(str(key	))
		command_queue = {}
		r.sleep()

if __name__ == '__main__':
	main()
