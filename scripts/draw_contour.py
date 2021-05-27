#!/usr/bin/env python
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from moveit_commander import PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import CollisionObject
import rospy
import numpy as np


def got_to_pose_straight_line(pose_goal, rot_goal=[0.0, 0.0, 0.0, 0.0], n_way_points=10):
    contour_list = []
    x_way_points = np.linspace(0, pose_goal[0], n_way_points)
    y_way_points = np.linspace(0, pose_goal[1], n_way_points)
    z_way_points = np.linspace(0, pose_goal[2], n_way_points)
    qx_way_points = np.linspace(0, rot_goal[0], n_way_points)
    qy_way_points = np.linspace(0, rot_goal[1], n_way_points)
    qz_way_points = np.linspace(0, rot_goal[2], n_way_points)
    qw_way_points = np.linspace(0, rot_goal[3], n_way_points)
    print (y_way_points)
    contour_point = Pose()
    for i in range(n_way_points):
        contour_point.position.x = x_way_points[i]
        contour_point.position.y = y_way_points[i]
        contour_point.position.z = z_way_points[i]
        contour_point.orientation.x = qx_way_points[i]
        contour_point.orientation.y = qy_way_points[i]
        contour_point.orientation.z = qz_way_points[i]
        contour_point.orientation.w = qw_way_points[i]
        contour_list.append(contour_point)
    return contour_list


def draw_rectangle(lenght_x, length_y, z_value, n_way_points=2):
    contour_list = []
    line_1 = [lenght_x, 0, z_value]
    line_2 = [lenght_x, length_y, z_value]
    line_3 = [0, length_y, z_value]
    line_4 = [0, 0, z_value]
    contour_list.extend(got_to_pose_straight_line([0, 0, z_value], [0, 0, 0, 0], n_way_points))
    contour_list.extend(got_to_pose_straight_line(line_1, [0, 0, 0, 0], n_way_points))
    contour_list.extend(got_to_pose_straight_line(line_2, [0, 0, 0, 0], n_way_points))
    contour_list.extend(got_to_pose_straight_line(line_3, [0, 0, 0, 0], n_way_points))
    contour_list.extend(got_to_pose_straight_line(line_4, [0, 0, 0, 0], n_way_points))
    return contour_list
  
  
rospy.init_node("draw_contour")
contour_pub = rospy.Publisher("contour_topic", PoseArray, queue_size=1)
rospy.sleep(1)

start_pose = PoseArray()
start_pose.header.frame_id = "base_link"
start_pose.header.seq = 0
start_pose.header.stamp = rospy.Time.now()
start_pose.poses.extend(got_to_pose_straight_line([-0.075, 0.125, 1.016], [1, 0, 0, 0], 10))
contour_pub.publish(start_pose)

raw_input()
contour_msg = PoseArray()
contour_msg.header.frame_id = "tool0_start"
contour_msg.header.seq = 0
contour_msg.header.stamp = rospy.Time.now()
contour_msg.poses.extend(got_to_pose_straight_line([0, 0, 0.003], n_way_points=2))
contour_msg.poses.extend(draw_rectangle(0.02, 0.02, 0.003, n_way_points=4))
contour_msg.poses.extend(got_to_pose_straight_line([0, 0, -0.03], n_way_points=2))
contour_pub.publish(contour_msg)

