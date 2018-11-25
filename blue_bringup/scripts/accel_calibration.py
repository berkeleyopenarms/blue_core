#!/usr/bin/env python

"""This node should run at startup, and sets the initial joint angles to some hardcoded values."""

import rospy
import pickle
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
from scipy.stats import multivariate_normal

# http://docs.ros.org/indigo/api/message_filters/html/python/#message_filters.TimeSynchronizer
# synch callbacks with this

# def measurement_likelihood(v_part, v_meas):
    # part = np.array([v_part.x(), v_part.y(), v_part.z()])
    # meas = np.array([v_meas.x, v_meas.y, v_meas.z])
    # return np.abs(part.dot(meas))

def measurement_likelihood(v_part, v_meas):
    part = np.array([v_part.x(), v_part.y(), v_part.z()])
    meas = np.array([v_meas.x, v_meas.y, v_meas.z])
    ml = multivariate_normal.pdf(part, mean=meas, cov=100.0*np.ones(3))
    return ml

class ParticleGroup:
    def __init__(self, num_joints, num_particles, joint_min, joint_max, fk):
        self.num_j = num_joints
        self.num_p = num_particles
        self.jmin = joint_min
        self.jmax = joint_max
        self.fk_solver = fk
        # uniformly initalize potential positions from given max and min
        self.particle_offsets = np.random.uniform(self.jmin, self.jmax, (self.num_p, self.num_j))
        # rospy.logerr(self.particle_offsets)

    def update(self, joints, accel_1, accel_2):
        measurement_likelihood_array = []
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
            accel_base = accel_1
            accel_end = accel_2

            grav_prev = kdl.Vector()
            grav_prev.x(accel_2.x)
            grav_prev.y(accel_2.y)
            grav_prev.z(accel_2.z)

            # transform the gravity vector by the given frame
            grav_post = frame * grav_prev

            # append the measurement likelyhood
            ml = measurement_likelihood(grav_post, accel_base)
            measurement_likelihood_array.append(ml)


        # normalize measurement likelyhood and resample particles
        alpha = np.sum(measurement_likelihood_array)
        measurement_likelihood_array= np.array(measurement_likelihood_array) / alpha
        cdf = np.cumsum(measurement_likelihood_array)

        # TODO verify that this is done correctly
        self.particle_offsets = np.array([self.particle_offsets[np.argwhere(cdf>np.random.uniform())[0,0],:] for i in range(self.num_p)])

        # TODO implement roughening
        if True:
            mean = np.zeros(self.num_j)
            sigma = 0.0001 * 1.0 * self.num_p**(-1.0/3.0) * np.ones((self.num_j, self.num_j))
            noise = np.random.multivariate_normal(mean, sigma, self.num_p)
            self.particle_offsets = self.particle_offsets + noise
        index = np.argmax(measurement_likelihood_array)
        return self.particle_offsets[index,:], measurement_likelihood_array[index]

class BlueRobotFilter:
    def _setup(self):
        # load in ros parameters
        self.gt= np.array(rospy.get_param("blue_hardware/simple_startup_angles"))

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
        rospy.logerr(self.joint_names)

        # load in min and max joints
        self.jmin = []
        self.jmax = []
        for j in range(self.num_joints):
            joint_limit = self.robot_urdf.joint_map[self.joint_names[j]].limit
            self.jmin.append(joint_limit.lower)
            self.jmax.append(joint_limit.upper)
        rospy.logerr(self.jmin)
        rospy.logerr(self.jmax)


        self.accel_links.insert(0, self.baselink)
        self.best_estimate = np.zeros(self.num_joints)
        self.probability = 0

        # TODO make a rosparam
        particles_per_joint = 200

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
            self.history = []
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

        if self.debug:
            self.debug_count += 1
            all_particles = []
            for pg in self.pg_array:
                all_particles.append(pg.particle_offsets.copy())
            # rospy.logerr(all_particles)
            all_particles = np.hstack(all_particles)
            if self.debug_count&5 == 0:
                # rospy.logerr(all_particles.shape)
                rospy.logerr(len(self.history))
                self.history.append(all_particles)
                if len(self.history) == 200:
                    with open('/home/phil/fixed.pickle', 'wb') as handle:
                            pickle.dump(self.history, handle)

        self.best_estimate = np.array([item for sublist in best_estimate for item in sublist])
        self.probability = np.min(probabilities)

        rospy.logerr(np.array(self.best_estimate) - self.gt[0:7])
        # rospy.logerr(np.array(self.best_estimate) - np.array(new_joint_positions)[0:7])
        # rospy.logerr(probabilities)

    def calibrate(self):
        joint_state_subscriber = Subscriber("/joint_states", JointState)
        gravity_vector_subscriber = Subscriber("blue_hardware/gravity_vectors", GravityVectorArray)
        tss = ApproximateTimeSynchronizer([joint_state_subscriber, gravity_vector_subscriber], queue_size=50, slop=0.01)

        # register joint states and gravity vectors callback
        tss.registerCallback(self.update_filter)
#
    # def exit(self):
        # if self.debug:


if __name__ == "__main__":

    rospy.init_node("simple_startup_calibration")

    # building the kinematic chains
    rospy.loginfo("Building Blue Object...")
    blue = BlueRobotFilter(debug=True)
    # atexit.register(blue.exit)

    # start the calibration callback functions
    blue.calibrate()

    # wait until a high probability is reached
    r = rospy.Rate(1.0)
    start = rospy.get_rostime()
    while blue.get_probability() < 0.9 and not rospy.is_shutdown() and start + rospy.Duration.from_sec(60) > rospy.get_rostime():
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
