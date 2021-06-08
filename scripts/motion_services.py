#! /usr/bin/env python

from cut_contour.robot_helpers import filter_plan
from cut_contour.msg import MoveStraightAction, MoveStraightActionFeedback
from cut_contour.msg import MeasureZAction, MeasureZActionFeedback
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64
from math import fabs
import actionlib
import rospy
import roslib
roslib.load_manifest('cut_contour')


class MotionServer:
  def __init__(self, tool_group="cutting_tool"):
    self.group = MoveGroupCommander(tool_group)
    self.measure_z_server = actionlib.SimpleActionServer(
        'measure_z', MeasureZAction, self.measure_z, False)
    self.move_straight_server = actionlib.SimpleActionServer(
        'move_straight', MoveStraightAction, self.move_straight, False)
    self.move_straight_server.start()
  
  def compute_plan(self, goal):
    # set the pose_arr reference frame
    self.group.set_pose_reference_frame(goal.ref_frame)

    # compute the plan
    plan, fraction = self.group.compute_cartesian_path(
        goal.pose_array.poses, goal.eef_step, goal.jump_threshold, goal.avoid_collisions)
    print(fraction)

    # filter the output plan
    filtered_plan = filter_plan(plan)
    # filtered_plan = plan
    return filtered_plan

  def move_straight(self, goal):
    # Compute and filter the plan
    plan = self.compute_plan(goal)

    # Time paremetrize the path to generate trajectory.
    final_traj = self.group.retime_trajectory(
        self.group.get_current_state(), plan, goal.vel_scale, goal.acc_scale)

    # execute the filtered plan
    result = self.group.execute(final_traj)
    feedback = MoveStraightActionFeedback(curr_pose_idx=1)
    self.move_straight_server.publish_feedback(feedback)
    self.move_straight_server.set_succeeded(result=result, text="DONE!!")
  
  def move_to_touch(self, goal):
    # Compute and filter the plan
    plan = self.compute_plan(goal)

    # get the initial force
    msg = rospy.wait_for_message("/ft_sensor_wrench/filtered_z", Float64)
    init_force_z = msg.data
    print("initial_force = ", init_force_z)

    # execute the filtered plan
    final_traj = self.group.retime_trajectory(
        self.group.get_current_state(), plan, goal.vel_scale, goal.acc_scale)
    self.group.execute(final_traj, wait=False)

    current_force_z = init_force_z
    change_force = fabs(current_force_z - init_force_z)
    while change_force < self.force_z_threshold:
        msg = rospy.wait_for_message(
            "/ft_sensor_wrench/filtered_z", Float64)
        current_force_z = msg.data
        change_force = fabs(current_force_z - init_force_z)
        print("change in force = ", change_force)

    self.group.stop()


if __name__ == '__main__':
  rospy.init_node('motion_server')
  server = MotionServer()
  rospy.spin()
