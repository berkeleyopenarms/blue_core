#!/usr/bin/env python

"""This node provides a service that returns a set of joints that achieves the desired end effector pose."""

import rospy
import tf2_ros
import tf2_geometry_msgs
from trac_ik_python.trac_ik_wrap import TRAC_IK
from blue_msgs.srv import InverseKinematics, InverseKinematicsResponse
from geometry_msgs.msg import PoseStamped

class BlueIK:
    def __init__(self):
        # load in robot kinematic information
        self.baselink = rospy.get_param("blue_hardware/baselink")
        self.endlink = rospy.get_param("blue_hardware/endlink")
        self.posture_target = rospy.get_param("blue_hardware/posture_target")
        urdf = rospy.get_param("/robot_description")

        # set up tf2 for transformation lookups
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0)) # tf buffer length
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # build trac-ik solver
        self.ik = TRAC_IK(self.baselink,
                          self.endlink,
                          urdf,
                          0.01,
                          5e-4,
                          "Distance")
                          # "Manipulation2")
                          # "Manipulation1")

        rospy.Service('inverse_kinematics', InverseKinematics, self.handle_ik_request)

    def ik_solution(self, target_pose, seed_joint_positions, solver="trac-ik"):
        solver = solver.lower()

        # use the requested solver, default to trac-ik otherwise
        # currently only trac-ik is supported
        if solver == "trac-ik":
            ik_joints = self.ik.CartToJnt(seed_joint_positions,
                                       target_pose.position.x, target_pose.position.y, target_pose.position.z,
                                       target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
        else:
            ik_joints = self.ik.CartToJnt(seed_joint_positions,
                                       target_pose.position.x, target_pose.position.y, target_pose.position.z,
                                       target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)

        if not len(ik_joints) == len(seed_joint_positions):
            return False
        return ik_joints

    def handle_ik_request(self, request):
        if len(request.seed_joint_positions) == len(self.posture_target):
            seed_joint_positions = request.seed_joint_positions
        else:
            seed_joint_positions = self.posture_target

        try:
            # apply lookup transformation desired pose and apply the transformation
            transform = self.tf_buffer.lookup_transform(self.baselink,
                                                   request.end_effector_pose.header.frame_id,
                                                   rospy.Time(0), # get the tf at first available time
                                                   rospy.Duration(0.1)) # wait for 0.1 second
            pose_in_baselink_frame = tf2_geometry_msgs.do_transform_pose(request.end_effector_pose, transform)

            # get the ik solution
            ik_joints = self.ik_solution(pose_in_baselink_frame.pose, seed_joint_positions)

            if ik_joints == False:
                return InverseKinematicsResponse(False, [])
            else:
                return InverseKinematicsResponse(True, ik_joints)
        except:
            return InverseKinematicsResponse(False, [])


if __name__ == "__main__":
    rospy.init_node("simple_startup_calibration")
    blue_ik = BlueIK()
    rospy.spin()
