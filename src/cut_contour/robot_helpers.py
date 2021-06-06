import tf
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
import rospy
import numpy as np
from copy import deepcopy
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Float64
from math import fabs


def transform_poses(target_frame, source_frame, pose_arr):
    """
    Transform poses from source_frame to target_frame
    """
    transformer_listener = tf.TransformListener()
    trans_pose_arr = PoseArray()
    for i in range(len(pose_arr.poses)):
        trans_pose = PoseStamped()
        pose = PoseStamped()
        pose.header.frame_id = source_frame
        pose.pose = pose_arr.poses[i]
        transformer_listener.waitForTransform(
            target_frame, source_frame, rospy.Time(), rospy.Duration(1))
        trans_pose = transformer_listener.transformPose(
            target_frame, pose)
        trans_pose_arr.poses.append(trans_pose.pose)

    trans_pose_arr.header.frame_id = target_frame
    trans_pose_arr.header.stamp = rospy.Time()
    return trans_pose_arr

def sample_from_func(func, start, stop, number_of_points):
    func_input = np.linspace(start=start, stop=stop, num=number_of_points).tolist()
    func_output = [func(i) for i in func_input]
    return func_input, func_output

class MotionServices():
    def __init__(self, tool_group="cutting_tool") -> None:
        self.move_group = MoveGroupCommander(tool_group)
        # TODO: Adjust quaternions of links
        self.tool_quat_base_link = [0, 1, 0, 0]
        self.tool_quat_table_link = [0.707, -0.707, 0.000, -0.000]
        self.approach_dist = 0.01
        self.retreat_feed_rate = 0.5
        self.measure_z_feed_rate = 0.0005
        self.retreat_dist = 0.03
        self.z_to_measure = -0.03
        self.force_z_threshold = 1.0
    
    def approch_point(self, point_location, ref_frame="base_link"):

        self.move_group.set_pose_reference_frame(ref_frame)
        # approach
        set_point = Pose()
        set_point.position.x = point_location.position.x
        set_point.position.y = point_location.position.y
        set_point.position.z = point_location.position.z + self.approach_dist

        quat = self.tool_quat_base_link

        if ref_frame == "table_link":
            quat = self.tool_quat_table_link

        set_point.orientation.x = quat[0]
        set_point.orientation.y = quat[1]
        set_point.orientation.z = quat[2]
        set_point.orientation.w = quat[3]

        self.move_group.clear_pose_targets()
        self.move_group.set_pose_target(set_point)
        approach_result = self.move_group.go()
        return approach_result
    
    def move_straight(self, point_location, z_dist=0, ref_frame="base_link", vel_scale=1, acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0):
        # set the pose_arr reference frame
        self.move_group.set_pose_reference_frame(ref_frame)

        # compute the plan
        goal_pose_arr = []
        set_point = deepcopy(point_location)
        set_point.position.x = point_location.position.x
        set_point.position.y = point_location.position.y
        set_point.position.z = point_location.position.z + z_dist

        goal_pose_arr.append(set_point)
        plan, fraction = self.move_group.compute_cartesian_path(
            goal_pose_arr, eef_step, jump_threshold, avoid_collisions)
        print(fraction)

        # filter the output plan
        filtered_plan = self.__filter_plan(plan)
        # filtered_plan = plan

        # execute the filtered plan
        final_traj = self.move_group.retime_trajectory(
            self.move_group.get_current_state(), filtered_plan, vel_scale, acc_scale)
        result = self.move_group.execute(final_traj)
        return result
    
    def __filter_plan(self, plan):
        last_time_step = plan.joint_trajectory.points[0].time_from_start.to_sec
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header = plan.joint_trajectory.header
        new_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
        new_plan.joint_trajectory.points.append(
            plan.joint_trajectory.points[0])
        for i in range(1, len(plan.joint_trajectory.points)):
            point = plan.joint_trajectory.points[i]
            if point.time_from_start.to_sec > last_time_step:
                new_plan.joint_trajectory.points.append(point)
            last_time_step = point.time_from_start.to_sec
        return new_plan
    
    def measure_z(self, point_location, ref_frame="base_link", vel_scale=1, acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0):
        # set the pose_arr reference frame
        self.move_group.set_pose_reference_frame(ref_frame)

        # compute the plan
        goal_pose_arr = []
        set_point = Pose()
        set_point.position.x = point_location.position.x
        set_point.position.y = point_location.position.y
        set_point.position.z = point_location.position.z + self.z_to_measure

        quat = self.tool_quat_base_link
        if ref_frame == "table_link":
            quat = self.tool_quat_table_link
        set_point.orientation.x = quat[0]
        set_point.orientation.y = quat[1]
        set_point.orientation.z = quat[2]
        set_point.orientation.w = quat[3]

        goal_pose_arr.append(set_point)
        plan, fraction = self.move_group.compute_cartesian_path(
            goal_pose_arr, eef_step, jump_threshold, avoid_collisions)
        print(fraction)

        # filter the output plan    def cut_cb(self, contour_msg):
        filtered_plan = self.__filter_plan(plan)

        # get the initial force
        msg = rospy.wait_for_message("/ft_sensor_wrench/filtered_z", Float64)
        init_force_z = msg.data
        print("initial_force = ", init_force_z)

        # execute the filtered plan
        final_traj = self.move_group.retime_trajectory(
            self.move_group.get_current_state(), filtered_plan, vel_scale, acc_scale)
        self.move_group.execute(final_traj, wait=False)

        current_force_z = init_force_z
        change_force = fabs(current_force_z - init_force_z)
        while change_force < self.force_z_threshold:
            msg = rospy.wait_for_message(
                "/ft_sensor_wrench/filtered_z", Float64)
            current_force_z = msg.data
            change_force = fabs(current_force_z - init_force_z)
            print("change in force = ", change_force)

        self.move_group.stop()
        # get the current z
        pose_arr = PoseArray()
        tool_pose = PoseArray()
        pose = Pose()
        print(pose)
        pose_arr.header.frame_id = "tool0_comp"
        pose_arr.poses.append(pose)
        tool_pose = transform_poses("base_link", "tool0_comp", pose_arr)

        return tool_pose.poses[0].position.z
    
