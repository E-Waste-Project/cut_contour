import tf
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
import rospy
import numpy as np


tool_quat_base_link = [0, 1, 0, 0]
tool_quat_table_link = [0.707, -0.707, 0.000, -0.000]

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


class MotionInterace():
    def __init__(self) -> None:
        pass
def move_straight(self, point_location, z_dist=0, ref_frame="base_link", vel_scale=1, acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0):
    # set the pose_arr reference frame
    self.group.set_pose_reference_frame(ref_frame)

    # compute the plan
    goal_pose_arr = []
    set_point = Pose()
    set_point.position.x = point_location.position.x
    set_point.position.y = point_location.position.y
    set_point.position.z = point_location.position.z + z_dist
    quat = self.tool_quat_base_link
    if ref_frame == "table_link":
        quat = self.tool_quat_table_link
    set_point.orientation.x = quat[0]
    set_point.orientation.y = quat[1]
    set_point.orientation.z = quat[2]
    set_point.orientation.w = quat[3]

    goal_pose_arr.append(set_point)
    plan, fraction = self.group.compute_cartesian_path(
        goal_pose_arr, eef_step, jump_threshold, avoid_collisions)
    print(fraction)

    # filter the output plan
    filtered_plan = self.__filter_plan(plan)
    # filtered_plan = plan

    # execute the filtered plan
    final_traj = self.group.retime_trajectory(
        self.group.get_current_state(), filtered_plan, vel_scale, acc_scale)
    result = self.group.execute(final_traj)
    return result
    
