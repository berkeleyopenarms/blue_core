#!/usr/bin/env python

"""This node should run at startup, and allows users to request the IK solution."""

import rospy
import tf2_ros
import tf2_geometry_msgs
from trac_ik_python.trac_ik_wrap import TRAC_IK
from blue_msgs.srv import InverseKinematics, InverseKinematicsResponse
from geometry_msgs.msg import PoseStamped

class BlueIK:
    def __init__(self):
        # load in ROS parameters
        self.baselink = rospy.get_param("blue_hardware/baselink")
        self.endlink = rospy.get_param("blue_hardware/endlink")
        self.posture_target = rospy.get_param("blue_hardware/posture_target")
        urdf = rospy.get_param("/robot_description")

        # set up tf2
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Build ik solver
        self.ik = TRAC_IK(self.baselink,
                          self.endlink,
                          urdf,
                          0.01,
                          5e-4,
                          "Distance")
                          # "Manipulation2")
                          # "Manipulation1")

        rospy.Service('inverse_kinematics', InverseKinematics, self.handle_ik_request)

    def ik_solution(self, target, joints):
        # target is a target pose object
        # joints is the starting joint seed
        seed_state = []
        for j in joints:
            seed_state.append(j)

        result = self.ik.CartToJnt(seed_state,
                                   target.position.x, target.position.y, target.position.z,
                                   target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w)

        if not len(result) == self.num_joints:
            return False
        return result

    def handle_ik_request(self, request):
        if len(request.seed_joint_positions) == len(self.posture_target):
            seed_joints = request.seed_joint_positions
        else:
            seed_joints = self.posture_target

        try:
            # apply lookup transformation desired pose and apply the transformation
            transform = tf_buffer.lookup_transform(self.baselink,
                                                   request.end_effector_pose.header.frame_id,
                                                   rospy.Time(0), #get the tf at first available time
                                                   rospy.Duration(0.1)) #wait for 0.1 second
            pose_in_baselink_frame = tf2_geometry_msgs.do_transform_pose(request.end_effector_pose, transform)

            if request.solver == "trac-ik":
                ik_joints = self.ik_solution(pose_in_baselink_frame.pose, seed_joints)
            else:
                ik_joints = self.ik_solution(pose_in_baselink_frame.pose, seed_joints)

            if ik_joints == False:
                return InverseKinematicsResponse([], False)
            else:
                return InverseKinematicsResponse(ik_joints, False)
        except:
            return InverseKinematicsResponse([], False)


if __name__ == "__main__":
    rospy.init_node("simple_startup_calibration")
    blue_ik = BlueIK()
    rospy.spin()
