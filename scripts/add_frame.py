#!/usr/bin/env python  
import rospy
import tf
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import TransformStamped, Transform, PointStamped


rospy.init_node('fixed_tf_broadcaster')

new_frame = "camera_start"
attached_frame = "tool0"
ref_frame = "base_link"

br = tf.TransformBroadcaster()
transformer_listener = tf.TransformListener()
transformer_listener.waitForTransform(ref_frame, attached_frame, rospy.Time(),rospy.Duration(10))
trans, rot = transformer_listener.lookupTransform(ref_frame, attached_frame ,rospy.Time())

rate = rospy.Rate(50.0)

while not rospy.is_shutdown():
    br.sendTransform(trans, rot, rospy.Time().now(), new_frame, ref_frame)
    rate.sleep()