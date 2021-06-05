#!/usr/bin/env python
from struct import pack
import rospy
from geometry_msgs.msg import PoseArray, Pose

from copy import deepcopy
from robot_helpers import transform_poses, sample_from_func
from math import sqrt

class FlippingPlanner():
    def __init__(self, flip_dist=1) -> None:
        self.flip_point = None
        self.center_point = None
        self.flip_radius = None
        self.flip_dist = flip_dist
        rospy.Subscriber("/flip_xyz", PoseArray, self.flip_cb)

    def flip_cb(self, flip_data_msg):
        trans_flip_data = transform_poses("base_link", "calibrated_frame", flip_data_msg)
        self.flip_point = trans_flip_data.poses[0]
        self.center_point = trans_flip_data.poses[1]
        self.flip_radius = self.center_point.position.y - self.flip_point.position.y
        y_arr, z_arr = sample_from_func(self.circle_func, self.flip_point.position.y, self.center_point.position.y + self.flip_dist, 20)
        plan = PoseArray()
        for y, z in zip(y_arr, z_arr):
            pose_goal = Pose()
            pose_goal.position.x = self.flip_point.position.x
            pose_goal.position.y = y
            pose_goal.position.z = z
            # TODO: Adjust Quaternion values.
            pose_goal.orientation.x = 0
            pose_goal.orientation.y = 1
            pose_goal.orientation.z = 0
            pose_goal.orientation.w = 0
            plan.poses.append(pose_goal)
        
    
    def circle_func(self, y):
        return sqrt(self.flip_radius**2 - (y - self.center_point.position.y)**2) + self.center_point.position.z
        
