import tf
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
import rospy
import numpy as np
from copy import deepcopy
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Float64
from math import fabs



def sample_from_func(func, start, stop, number_of_points):
    func_input = np.linspace(start=start, stop=stop, num=number_of_points).tolist()
    func_output = [func(i) for i in func_input]
    return func_input, func_output

def filter_plan(plan):
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


class TransformServices():
    def __init__(self):
        self.transformer_listener = tf.TransformListener()


        
    def transform_poses(self, target_frame, source_frame, pose_arr):
        """
        Transform poses from source_frame to target_frame
        """
        trans_pose_arr = PoseArray()
        for i in range(len(pose_arr.poses)):
            trans_pose = PoseStamped()
            pose = PoseStamped()
            pose.header.frame_id = source_frame
            pose.pose = pose_arr.poses[i]
            self.transformer_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(), rospy.Duration(1))
            trans_pose = self.transformer_listener.transformPose(
                target_frame, pose)
            trans_pose_arr.poses.append(trans_pose.pose)

        trans_pose_arr.header.frame_id = target_frame
        trans_pose_arr.header.stamp = rospy.Time()
        return trans_pose_arr
    
    def lookup_transform(self, target_frame, source_frame):
        self.transformer_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(), rospy.Duration(1))
        t, r = self.transformer_listener.lookupTransform(target_frame, source_frame, rospy.Time())
        pose = Pose()
        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]
        pose.orientation.x = r[0]
        pose.orientation.y = r[1]
        pose.orientation.z = r[2]
        pose.orientation.w = r[3]
        return pose


class MotionServices():
    def __init__(self, tool_group="cutting_tool"):
        self.move_group = MoveGroupCommander(tool_group)
        # TODO: Adjust quaternions of links
        self.tool_quat_base_link = [0, 1, 0, 0]
        self.tool_quat_table_link = [0.707, -0.707, 0.000, -0.000]
        self.current_force_z = 0
        self.current_force_xy = 0
        rospy.Subscriber("ft_sensor_wrench/resultant/filtered", Float64, self.force_xy_cb)
        rospy.Subscriber("ft_sensor_wrench/filtered_z", Float64, self.force_z_cb)
    
    def move_straight(self, poses, ref_frame="base_link",
                      vel_scale=0.005, acc_scale=0.005,
                      avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0,
                      wait_execution=True):
        # set the pose_arr reference frame
        self.move_group.set_pose_reference_frame(ref_frame)

        plan, fraction = self.move_group.compute_cartesian_path(
            poses.poses, eef_step, jump_threshold, avoid_collisions)
        print(fraction)

        # filter the output plan    def cut_cb(self, contour_msg):
        filtered_plan = filter_plan(plan)
        
        # execute the filtered plan
        final_traj = self.move_group.retime_trajectory(
            self.move_group.get_current_state(), filtered_plan, vel_scale, acc_scale)
        
        result = self.move_group.execute(final_traj, wait_execution)
        
        return result
    
    def force_z_cb(self, msg):
        self.current_force_z = msg.data
    
    def force_xy_cb(self, msg):
        self.current_force_xy = msg.data
    
    def move_to_touch(self, poses, axis, force_thersh=0.5, ref_frame="base_link",
                      vel_scale=0.005, acc_scale=0.005,
                      avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0):

        # get the initial force
        init_force = self.current_force_xy if axis == 'z' else self.current_force_z
        current_force = init_force
        change_force = 0
        # rospy.loginfo("initial_force = {}".format(init_force))
        
        # Move fast at first.
        result = self.move_straight(poses, vel_scale=vel_scale, acc_scale=acc_scale, wait_execution=False)

        # Monitor the force until it reaches force_thresh.
        while change_force < force_thersh:
            current_force = self.current_force_xy if axis == 'z' else self.current_force_z
            change_force = fabs(current_force - init_force)
            # rospy.loginfo("change in force = {}".format(change_force))
        
        # rospy.loginfo("Initial Time")
        self.move_group.stop()
    
