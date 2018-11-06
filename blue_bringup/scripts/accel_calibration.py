#!/usr/bin/env python

"""This node should run at startup, and sets the initial joint angles to some hardcoded values."""

import rospy
import sys
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
from blue_msgs.srv import JointStartupCalibration

class ParticleFilter:
    def __init__(self, num_joints, differential_pairs, ):


class particle_group:
    def __init__(self, num_joints, num_particles, joint_min, joint_max):
        self.num_j = num_joints
        self.num_p = num_particles
        self.jmin = joint_min
        self.jmax = joint_max
        self.particle_offsets = np.uniform.random(self.jmin, self.jmax, (num_p, num_j))

class BlueRobotFilter:
    def _setup(self):
        # load in ros parameters
        self.baselink = rospy.get_param("blue_hardware/baselink")
        self.endlink = rospy.get_param("blue_harware/endlink")
        flag, self.tree = kdl_parser.treeFromParam("/robot_description")

        self.joint_names = rospy.get_param("blue_hardware/joint_names")
        print self.joint_names

        # build kinematic chain and fk and jacobian solvers
        chain_ee = self.tree.getChain(self.baselink, self.endlink)
        self.fk_ee = kdl.ChainFkSolverPos_recursive(chain_ee)
        self.jac_ee = kdl.ChainJntToJacSolver(chain_ee)

        # building robot joint state
        self.num_joints = chain_ee.getNrOfJoints()
        self.joints = kdl.JntArray(self.num_joints)

    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            self.debug_count = 0
        self._setup()

    def update_joints(self, joint_msg):
        new_joints = self.joints.copy()
        for i, n in enumerate(self.joint_names):
            index = joint_msg.name.index(n)
            self.joints[i] = joint_msg.position[index]

    def calibrate(self):
        rospy.Subscriber("/joint_states", JointState, self.update_joints)

if __name__ == "__main__":
    rospy.init_node("simple_startup_calibration")

    # initalize filter


    # building the kinematic chains
    rospy.loginfo("Building Blue Object...")
    blue = BlueRobotFilter(debug=True)
    blue.calibrate()
    r = rospy.Rate(1.0)
    while blue.probability() < 0.9:
        r.sleep();

    # Read startup angles from parameter server
    startup_positions = blue.get_best_estimate();

    # Wait for calibration service to come up
    rospy.loginfo("Waiting for calibration service...")
    rospy.wait_for_service('blue_hardware/joint_startup_calibration')

    # Calibrate joints with startup angles
    rospy.loginfo("Starting calibration...")
    try:
        joint_startup_calibration = rospy.ServiceProxy('blue_hardware/joint_startup_calibration', JointStartupCalibration)
        response = joint_startup_calibration(startup_positions)

        if response.success:
            rospy.loginfo("Joint startup calibration succeeded!")
        else:
            rospy.logerr("Joint startup calibration failed!")

    except rospy.ServiceException as e:
        rospy.logerr("Joint startup calibration failed: %s" % e)


