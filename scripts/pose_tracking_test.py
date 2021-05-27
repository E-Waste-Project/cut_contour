#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node("pose_tracking")
pub = rospy.Publisher("/servo_server/target_pose", PoseStamped, queue_size=1, latch=True)

rospy.sleep(1)
rate = rospy.Rate(10.0)
pose_msg = PoseStamped()
# while not rospy.is_shutdown():
pose_msg.header.stamp = rospy.Time().now()
pose_msg.header.frame_id = 'base_link'
pose_msg.pose.position.x = 0.3
pose_msg.pose.position.y = 0
pose_msg.pose.position.z = 0.2
pose_msg.pose.orientation.x = 0
pose_msg.pose.orientation.y = 1
pose_msg.pose.orientation.z = 0
pose_msg.pose.orientation.w = 0
pub.publish(pose_msg)
rospy.sleep(1)
    