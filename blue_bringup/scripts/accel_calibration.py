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
from blue_msgs.msg import GravityVectorArray
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
import numpy as np
from urdf_parser_py import urdf

# http://docs.ros.org/indigo/api/message_filters/html/python/#message_filters.TimeSynchronizer
# synch callbacks with this

def measurement_likelihood(v_part, v_meas):
    part = np.array([v_part.x(), v_part.y(), v_part.z()])
    meas = np.array([v_meas.x, v_meas.y, v_meas.z])
    return np.abs(part.dot(meas))

class ParticleGroup:
    def __init__(self, num_joints, num_particles, joint_min, joint_max, fk):
        self.num_j = num_joints
        self.num_p = num_particles
        self.jmin = joint_min
        self.jmax = joint_max
        self.fk_solver = fk
        # uniformly initalize potential positions from given max and min
        self.particle_offsets = np.random.uniform(self.jmin, self.jmax, (self.num_p, self.num_j))
        rospy.logerr(self.particle_offsets)

    def update(self, joints, accel_1, accel_2):
        ml_array = []
        kdl_joints = kdl.JntArray(self.num_j)
        frame = kdl.Frame()

        # for each particle (row joint offset) check the likelyhood given the measurement
        for p in self.particle_offsets:
            # get the joints angle with offset for the given particle
            corrected_joints = np.array(joints) + p
            for j in range(self.num_j):
                kdl_joints[j] = corrected_joints[j]
            self.fk_solver.JntToCart(kdl_joints, frame)
            # find the frame transformation

            grav_prev = kdl.Vector()
            grav_prev.x(accel_1.x)
            grav_prev.y(accel_1.y)
            grav_prev.z(accel_1.z)

            # transform the gravity vector by the given frame
            grav_post = frame * grav_prev

            # append the measurement likelyhood
            ml = measurement_likelihood(grav_post, accel_2)
            ml_array.append(ml)


        # normalize measurement likelyhood and resample particles
        alpha = np.sum(ml_array)
        ml_array = np.array(ml_array) / alpha
        cdf = np.cumsum(ml_array)

        # TODO verify that this is done correctly
        self.particle_offsets = np.array([self.particle_offsets[np.argwhere(cdf>np.random.uniform())[0,0],:] for i in range(self.num_p)])

        # TODO implement roughening
        if False:
            pass

        index = np.argmax(ml_array)
        return self.particle_offsets[index,:], ml_array[index]

class BlueRobotFilter:
    def _setup(self):
        # load in ros parameters
        self.baselink = rospy.get_param("blue_hardware/baselink")
        self.endlink = rospy.get_param("blue_hardware/endlink")
        robot_description = rospy.get_param("/robot_description")
        flag, self.tree = kdl_parser.treeFromString(robot_description)

        # build kinematic chain and fk and jacobian solvers
        self.robot_urdf = urdf.Robot.from_xml_string(robot_description)
        chain_ee = self.tree.getChain(self.baselink, self.endlink)

        # building robot joint state
        self.num_joints = chain_ee.getNrOfJoints()

        # other joint information
        self.joint_names = rospy.get_param("blue_hardware/joint_names")
        self.accel_links = rospy.get_param("blue_hardware/accel_links")
        self.differential_pairs = rospy.get_param("blue_hardware/differential_pairs")

        # load in min and max joints
        self.jmin = []
        self.jmax = []
        for j in range(self.num_joints):
            joint_limit = self.robot_urdf.joint_map[self.joint_names[j]].limit
            self.jmin.append(joint_limit.lower)
            self.jmax.append(joint_limit.upper)


        self.accel_links.insert(0, self.baselink)
        self.best_estimate = np.zeros(self.num_joints)
        self.probability = 0

        # TODO make a rosparam
        particles_per_joint = 100

        ppj = particles_per_joint
        p_groups = []
        joints_per_group = []
        i = 0
        g = 0

        # build chain and filter between each gravity vector
        while i < self.num_joints:
            # get chain between two accel_links and build respective fk solver
            chain = self.tree.getChain(self.accel_links[g], self.accel_links[g+1])
            rospy.logerr("pair: " + self.accel_links[g] +", " + self.accel_links[g+1])
            fk = kdl.ChainFkSolverPos_recursive(chain)

            # build particle groups
            if i in self.differential_pairs:
                pg = ParticleGroup(2, ppj, [self.jmin[i], self.jmin[i+1]], [self.jmax[i], self.jmax[i+1]], fk)
                p_groups.append(pg)
                joints_per_group.append(2)
                i = i + 1
            else:
                pg = ParticleGroup(1, ppj, [self.jmin[i]], [self.jmax[i]], fk)
                p_groups.append(pg)
                joints_per_group.append(1)
            i = i + 1
            g = g + 1

        self.pg_array = p_groups
        self.jpg = joints_per_group

    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            self.debug_count = 0
        self._setup()

    def get_probability(self):
        return self.probability

    def get_best_estimate(self):
        return self.best_estimate

    def update_filter(self, joints_msg, accel_msg):

        # retrive joint angles in correct order from joint state message (comes unordered)
        new_joint_positions = []
        for jn in self.joint_names:
            index = joints_msg.name.index(jn)
            new_joint_positions.append(joints_msg.position[index])

        i = 0
        probabilities = []
        best_estimate = []
        for j, pg in enumerate(self.pg_array):
            # for each particle filter group querry the relevent joint angles
            new_link_joints = []
            for k in range(self.jpg[j]):
                new_link_joints.append(new_joint_positions[i])
                i = i + 1
            # update the filter with new accelerometer values and joint angles
            joints, prob = pg.update(new_link_joints, accel_msg.vectors[j], accel_msg.vectors[j+1])
            probabilities.append(prob)
            best_estimate.append(joints)

        self.best_estimate = np.array([item for sublist in best_estimate for item in sublist])
        self.probability = np.min(probabilities)

        # rospy.logerr(best_estimate)
        # rospy.logerr(probabilities)

    def calibrate(self):
        joint_state_subscriber = Subscriber("/joint_states", JointState)
        gravity_vector_subscriber = Subscriber("blue_hardware/gravity_vectors", GravityVectorArray)
        tss = ApproximateTimeSynchronizer([joint_state_subscriber, gravity_vector_subscriber], queue_size=2, slop=0.05)

        # register joint states and gravity vectors callback
        tss.registerCallback(self.update_filter)

if __name__ == "__main__":
    rospy.init_node("simple_startup_calibration")

    # building the kinematic chains
    rospy.loginfo("Building Blue Object...")
    blue = BlueRobotFilter(debug=True)

    # start the calibration callback functions
    blue.calibrate()

    # wait until a high probability is reached
    r = rospy.Rate(1.0)
    while blue.get_probability() < 0.9:
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
