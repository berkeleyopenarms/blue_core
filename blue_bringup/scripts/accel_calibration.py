#!/usr/bin/env python
import rospy
import sys
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from blue_msgs.msg import MotorState
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser


class AccelerometerCalibrator:
    def _setup(self):
        rospy.logerr(rospy.get_namespace())
        self.baselink = rospy.get_param("blue_hardware/baselink")
        self.endlink = rospy.get_param("blue_hardware/endlink")
        flag, self.tree = kdl_parser.treeFromParam("/robot_description")

        chain_ee = self.tree.getChain(self.baselink, self.endlink)
        self.fk_ee = kdl.ChainFkSolverPos_recursive(chain_ee)

        # to do set up intermediary chains
        chain_ee = self.tree.getChain(self.baselink, self.endlink)
        self.fk_ee = kdl.ChainFkSolverPos_recursive(chain_ee)



        frame = kdl.Frame()
        joints = kdl.JntArray(chain_ee.getNrOfJoints())
        cart = self.fk_ee.JntToCart(joints, frame)
        print(frame.p)
        print(frame.M)


    def __init__(self):
        rospy.init_node("accel_calibrator")
        self._setup()


        # command_publisher = rospy.Publisher("pose_target/command", PoseStamped, queue_size=1)
        # rospy.Subscriber("controller_pose", PoseStamped, self.command_callback, queue_size=1)
        # rospy.Subscriber("command_label", Int32, label_callback, queue_size=1)
        rospy.Subscriber("joint_imu_0", Imu, label_callback, queue_size=1)
        rospy.Subscriber("joint_imu_1", Imu, label_callback, queue_size=1)
        rospy.Subscriber("joint_imu_2", Imu, label_callback, queue_size=1)
        rospy.Subscriber("joint_imu_3", Imu, label_callback, queue_size=1)
        rospy.Subscriber("joint_imu_4", Imu, label_callback, queue_size=1)
        # gc = GripperClient()
        # gc.clear()
        # gc.set_effort(2.5)


    def command_callback(self, msg):
        if cmd_label == 1:
            command_publisher.publish(msg)

class ParicleFilter:
    def __init__(self, num_particles, dynamics_chains):
        self.init_particles(num_particles)

    # def init_particles():


def main():
    ac = AccelerometerCalibrator()
    rospy.spin()

if __name__ == "__main__":
    main()
