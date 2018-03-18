#!/usr/bin/env python
from comms import BLDCControllerClient
import time
import serial
import math
import signal
import sys
import rospy
import json
import termios
import fcntl
import array
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from koko_hardware_drivers.msg import MotorState
from comms import *

class BLDCDriverNode:
    MAX_CURRENT = 1.5
    MAX_TEMP_WARN = 55 # degrees C
    MAX_TEMP_MOTORS_OFF = 75
    CONTROL_LOOP_FREQ = 200

    def __init__(self):
        rospy.init_node('bldc_driver', anonymous=True)

        motor_ids = rospy.get_param('motor_ids')
        motor_names = rospy.get_param('motor_names')
        self.motor_names = {}
        for id, name in zip(motor_ids, motor_names):
            self.motor_names[id] = name

        port = sys.argv[1]
        self.serial = serial.Serial(port=port, baudrate=1000000, timeout=0.1)
        self.bldc = BLDCControllerClient(self.serial)
        self.command_queue = {}
        self.starting_angles = {}
        self.stop_motors = False

        self.boot()
        self.calibrate()

        self.state_publisher = rospy.Publisher("motor_states", MotorState, queue_size=1)
        for motor_id in self.motor_names:
            rospy.Subscriber(self.motor_names[motor_id] + "_cmd", Float64, self.make_set_command(motor_id), queue_size=1)
        rospy.Subscriber("stop_motors", Bool, self.stop_motors_cb, queue_size=1)

        self.loop()

    def boot(self):
        # assert self.low_latency_mode(self.serial.fd)
        # Initial hardware setup
        for id in self.motor_names:
            rospy.loginfo("Booting motor %d..." % id)
            self.bldc.leaveBootloader(id)
            rospy.sleep(0.1)
            self.serial.flush()
        self.serial.flush()

    def calibrate(self):
        calibration_success = False
        # Write calibration values
        for id in self.motor_names:
            success = False
            for attempt in range(5):
                try:
                    rospy.loginfo("Calibrating motor %d..." % id)
                    calibrations = self.bldc.readCalibration(id)
                    print(calibrations)
                    self.bldc.setZeroAngle(id, calibrations['angle'])
                    self.bldc.setCurrentControlMode(id)
                    self.bldc.setInvertPhases(id, calibrations['inv'])
                    self.bldc.setERevsPerMRev(id, calibrations['epm'])
                    self.bldc.setTorqueConstant(id, calibrations['torque'])
                    # self.bldc.writeRegisters(id, 0x1023, 1, struct.pack('<f', calibrations['zero']))
                    self.starting_angles[id] = 0.0
                    rospy.loginfo("Motor %d ready: supply voltage=%fV", id, self.bldc.getVoltage(id))
                    success = True
                    break
                except Exception as e:
                    rospy.logwarn(str(e))
                    rospy.sleep(0.2)
                    self.serial.flush()
                    self.bldc.leaveBootloader(id)
                    rospy.sleep(0.2)
                    self.serial.reset_input_buffer()
            if not success:
                rospy.logerr("Could not calibrate motors")
                rospy.signal_shutdown("Could not calibrate motors")
                exit()

        rospy.loginfo("Successfully calibrated motors")

    def loop(self):
        r = rospy.Rate(self.CONTROL_LOOP_FREQ)
        while not rospy.is_shutdown():
            stateMsg = MotorState()
            for motor_id in self.motor_names:
                try:
                    if not self.stop_motors:
                        if motor_id in self.command_queue:
                            state = self.bldc.setCommandAndGetState(motor_id, self.command_queue.pop(motor_id))
                        else:
                            state = self.bldc.getState(motor_id)
                    else:
                        state = self.bldc.setCommandAndGetState(motor_id, 0.0)

                    angle, velocity, direct_current, quadrature_current, \
                            supply_voltage, temperature, accel_x, accel_y, accel_z = state

                    stateMsg.name.append(self.motor_names[motor_id])
                    stateMsg.position.append(angle)
                    stateMsg.velocity.append(velocity)
                    stateMsg.direct_current.append(direct_current)
                    stateMsg.quadrature_current.append(quadrature_current)
                    stateMsg.supply_voltage.append(supply_voltage)
                    stateMsg.temperature.append(temperature)

                    accel = Vector3()
                    accel.x = accel_x
                    accel.y = accel_y
                    accel.z = accel_z
                    stateMsg.accel.append(accel)

                    if temperature > self.MAX_TEMP_WARN:
                        rospy.logwarn_throttle(1, "Motor {} is overheating, currently at  {}C".format(motor_id, temperature))
                        if temperature > self.MAX_TEMP_MOTORS_OFF:
                            self.stop_motors = True
                            rospy.logerr("Motor %d is too hot, setting motor currents to zero", motor_id)

                except Exception as e:
                    rospy.logerr("Motor " + str(motor_id) +  " driver error: " + str(e))

            self.state_publisher.publish(stateMsg)
            r.sleep()

    def make_set_command(self, motor_id):
        def set_command(motor_id, msg):
            effort_raw = msg.data
            effort_filtered = effort_raw

            if effort_filtered > self.MAX_CURRENT:
                effort_filtered = self.MAX_CURRENT
            elif effort_filtered < -self.MAX_CURRENT:
                effort_filtered = -self.MAX_CURRENT
            self.command_queue[motor_id] = effort_filtered
        return lambda msg: set_command(motor_id, msg)

    def stop_motors_cb(self, msg):
        if msg.data:
            self.stop_motors = True
        else:
            self.stop_motors = False

    def low_latency_mode(self, fd):
        buf = array.array('i', [0] * 32)
        try:
            fcntl.ioctl(fd, termios.TIOCGSERIAL, buf)
            buf[4] |= 1 << 13
            fcntl.ioctl(fd, termios.TIOCSSERIAL, buf)
            return True
        except IOError as e:
            raise Exception('Could not set low latency mode')
        return False

if __name__ == '__main__':
    BLDCDriverNode()
