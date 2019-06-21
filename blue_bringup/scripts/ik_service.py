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

        # apply lookup transformation desired pose and apply the transformation
        transform = tf_buffer.lookup_transform(target_frame,
                                                                                      pose_stamped_to_transform.header.frame_id, #source frame
                                                                                      rospy.Time(0), #get the tf at first available time
                                                                                      rospy.Duration(1.0)) #wait for 1 second

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

            
        ik_joints = self.ik_solution(request.end_effector_pose, seed_joints)
        if ik_joints == False:
            return joints
        else:
            return ik_joints


if __name__ == "__main__":
    rospy.init_node("simple_startup_calibration")
    blue_ik = BlueIK()
    rospy.spin()
