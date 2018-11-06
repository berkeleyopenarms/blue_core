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

# http://docs.ros.org/indigo/api/message_filters/html/python/#message_filters.TimeSynchronizer
# synch callbacks with this

# class ParticleFilter:
    # def __init__(self, num_joints, differential_pairs, particles_per_joint, jmin, jmax):
#
    # def update(self, new_joints, accel):
        # TODO, update ParticleGroups
        # return mle_joints, mle
def measurement_likelihood(vec_part, vec_meas):
    return np.linalg.norm(vec_part.dot(vec_meas))

class ParticleGroup:
    def __init__(self, num_joints, num_particles, joint_min, joint_max, fk):
        self.num_j = num_joints
        self.num_p = num_particles
        self.jmin = joint_min
        self.jmax = joint_max
        self.fk_solver = fk
        self.particle_offsets = np.uniform.random(self.jmin, self.jmax, (num_p, num_j))

    def update(self, joints, accel_1, accel_2):
        # TODO figure out accel message type
        ml_array = []
        for p in self.particle_offsets:
            frame = kdl.Frame()
            self.fk_solver.JntToCart(joints + p, frame)
            grav_prev = kdl.Vector()
            grav_prev.data = accel_1
            grav_post = frame * grav_prev
            ml = measurement_likelihood(grav_post, accel_2)
            ml_array.append(ml)

        # TODO reample particles and update particle filter offsets
        self.particle_offsets = np.random.uniform(self.jmin, self.jmax, (num_p, num_j))

        # TODO implement roughening
        # some code that adds noise

        # TODO return max joint angles, and max likelyhood probability
        return ml_joints, ml


class BlueRobotFilter:
    def _setup(self):
        # load in ros parameters
        self.baselink = rospy.get_param("blue_hardware/baselink")
        self.endlink = rospy.get_param("blue_harware/endlink")
        flag, self.tree = kdl_parser.treeFromParam("/robot_description")
        # build kinematic chain and fk and jacobian solvers
        chain_ee = self.tree.getChain(self.baselink, self.endlink)

        # building robot joint state
        self.num_joints = chain_ee.getNrOfJoints()

        # other joint information
        self.joint_names = rospy.get_param("blue_hardware/joint_names")
        self.accel_links = rospy.get_param("blue_hardware/accel_links").insert(0, self.baselink)
        self.accel_links.insert(0, self.baselink)
        print self.joint_names

        particles_per_joints = 100

####
        ppj = particles_per_joint
        p_groups = []
        joints_per_group = []
        i = 0
        g = 0
        while i < num_joints:
            if i in differential_pairs:
                chain = self.tree.getChain(self.accel_links[g], self.accel_links[g+1])
                fk = kdl.ChainFkSolverPos_recursive(chain)
                pg = ParticleGroup(2, ppj, [jmin[i], jmin[i+1]], [jmax[i], jmax[i+1]], fk)

                p_groups.append(pg)
                joints_per_group.append(2)
                i = i + 1
            else:
                chain = self.tree.getChain(self.accel_links[g], self.accel_links[g+1])
                fk = kdl.ChainFkSolverPos_recursive(chain)
                pg = ParticleGroup(1, ppj, [jmin[i]], [jmax[i]], fk)

                p_groups.append(pg)
                joints_per_group.append(1)
            i = i + 1
            g = g + 1
####

        self.pg_array = p_groups
        self.jpg = joints_per_group

    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            self.debug_count = 0
        self._setup()

    def update_joints(self, joint_msg):
        # TODO figure out how to synchronize with accelerometer
        pass

    def update_filter(self, joints, accel):
        new_joints = self.joints.copy()
        i = 0
        for j, pg in enumerate(self.pg_array):

            index = joint_msg.name.index(n)
            self.joints[i] = joint_msg.position[index]

    def calibrate(self):
        rospy.Subscriber("/joint_states", JointState, self.update_joints)

if __name__ == "__main__":
    rospy.init_node("simple_startup_calibration")

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
