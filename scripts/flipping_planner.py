#!/usr/bin/env python
from struct import pack
import rospy
from geometry_msgs.msg import PoseArray, Pose
import actionlib
from cut_contour.msg import MoveStraightAction, MoveStraightActionGoal
from cut_contour.msg import MeasureZAction, MeasureZActionGoal

from cut_contour.robot_helpers import TrasformServices, sample_from_func, MotionServices
from math import sqrt
from copy import deepcopy


class FlippingPlanner():
    def __init__(self, flip_dist=0.04):
        self.flip_point = None
        self.center_point = None
        self.flip_radius = None
        self.flip_dist = flip_dist
        rospy.Subscriber("/flip_xyz", PoseArray, self.old_way)
        self.measure_client = actionlib.SimpleActionClient(
            'measure_z', MeasureZAction)
        self.move_client = actionlib.SimpleActionClient(
            'move_straight', MoveStraightAction)
        self.motion_services = MotionServices("flipping")
        self.transform_services = TrasformServices()
        
    def old_way(self, flip_data_msg):
        print("recieved Flip Data")
        trans_flip_data = self.transform_services.transform_poses(
            "base_link", "calibrated_frame", flip_data_msg)
        self.flip_point = trans_flip_data.poses[0]
        self.center_point = trans_flip_data.poses[1]
                
        # go to flip pose   python server
        self.motion_services.move_group.set_named_target("flipping_pose")
        self.motion_services.move_group.go()
        
        # measure z
        poses = PoseArray()
        flip_pose = self.transform_services.lookup_transform("base_link", "flipping_upper")
        flip_pose.position.z -= 0.05
        poses.poses.append(flip_pose)
        self.motion_services.move_to_touch(poses=poses, axis='z')
        rospy.loginfo("Touched !!!!!")
        
        # move slightly upwards to avoid friction.
        poses = PoseArray()
        flip_pose = self.transform_services.lookup_transform("base_link", "flipping_upper")
        flip_pose.position.z += 0.0005
        poses.poses.append(flip_pose)
        self.motion_services.move_straight(poses)
        rospy.loginfo("Touched !!!!!")
        
        # get flip point by touching laptop
        poses = PoseArray()
        flip_pose = self.transform_services.lookup_transform("base_link", "flipping_upper")
        flip_pose.position.x += 0.1
        poses.poses.append(flip_pose)
        self.motion_services.move_to_touch(poses=poses, axis='xy', force_thersh=10)
        rospy.loginfo("Touched !!!!!")
        
        
        
        # generate plan
        poses = PoseArray()
        self.flip_point = self.transform_services.lookup_transform("base_link", "flipping_upper")
        self.center_point.position.x -= 0.04
        self.flip_radius = self.center_point.position.x - self.flip_point.position.x
        print("RADIUS = ", self.flip_radius)
        x_arr, z_arr = sample_from_func(
            self.circle_func, self.flip_point.position.x, self.center_point.position.x + self.flip_dist, 200)

        plan = PoseArray()
        half_arc_len = len(x_arr) // 2
        for x, z in zip(x_arr, z_arr):
            pose_goal = deepcopy(self.flip_point)
            pose_goal.position.x = x
            pose_goal.position.z = z
            plan.poses.append(pose_goal)
            
        first_half_plan = PoseArray()
        first_half_plan.poses = deepcopy(plan.poses[:half_arc_len])
        
        # execute first half plan
        self.motion_services.move_straight(first_half_plan, vel_scale=0.1, acc_scale=0.1)
        rospy.loginfo(" Half Arc Completed !!!!!")
        
        # Change tool orientation.
        orient_pose = deepcopy(plan.poses[half_arc_len])
        orient_pose.orientation.x = 0.681803123781
        orient_pose.orientation.y = -0.155651331771
        orient_pose.orientation.z = 0.127214182103
        orient_pose.orientation.w = 0.703373098144
        orient_poses = PoseArray()
        orient_poses.poses.append(orient_pose)
        
        self.motion_services.move_straight(orient_poses, vel_scale=0.1, acc_scale=0.1)
        rospy.loginfo("Oriented !!!!")
        
        # Continue the second half of the arc.
        second_half_plan = PoseArray()
        second_half_plan.poses = deepcopy(plan.poses[half_arc_len+1:])
        for pose in second_half_plan.poses:
            pose.orientation.x = 0.681803123781
            pose.orientation.y = -0.155651331771
            pose.orientation.z = 0.127214182103
            pose.orientation.w = 0.703373098144
        
        # execute plan
        self.motion_services.move_straight(second_half_plan, vel_scale=0.1, acc_scale=0.1)
        rospy.loginfo("Flipped !!!!!")
        
        # go to flip posee
        self.motion_services.move_group.set_named_target("flipping_pose")
        self.motion_services.move_group.go()
        
        
    def flip_cb(self, flip_data_msg):
        trans_flip_data = self.transform_services.transform_poses("base_link", "calibrated_frame", flip_data_msg)
        self.flip_point = trans_flip_data.poses[0]
        self.center_point = trans_flip_data.poses[1]
        self.flip_radius = self.center_point.position.x - self.flip_point.position.x
        x_arr, z_arr = sample_from_func(self.circle_func, self.flip_point.position.x, self.center_point.position.x + self.flip_dist, 100)
        
        plan = PoseArray()
        for x, z in zip(x_arr, z_arr):
            pose_goal = Pose()
            pose_goal.position.x = x
            pose_goal.position.y = self.flip_point.position.y
            pose_goal.position.z = z
            # TODO: Adjust Quaternion values.
            pose_goal.orientation.x = 0
            pose_goal.orientation.y = 1
            pose_goal.orientation.z = 0
            pose_goal.orientation.w = 0
            plan.poses.append(pose_goal)
        
        measure_plan = PoseArray()
        measure_plan.poses.append(plan.poses[0])
        measure_plan.poses[0].position.x -= 0.005
        measure_plan.poses[0].position.z -= 0.02
        
        self.move_client.wait_for_server()
        goal = MeasureZActionGoal(poses=measure_plan, ref_frame="base_link", vel_scale=1,
                                      acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()

        self.move_client.wait_for_server()
        goal = MoveStraightActionGoal(poses=plan, ref_frame="base_link", vel_scale=1,
                                      acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0)
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()
    
        return self.move_client.get_result()

    def circle_func(self, x):
        return sqrt(self.flip_radius**2 - (x - self.center_point.position.x)**2) + self.center_point.position.z
        
if __name__ == "__main__":
    rospy.init_node("Flipping_planner")
    flip_planner = FlippingPlanner()
    rospy.spin()
