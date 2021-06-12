#!/usr/bin/env python  
import rospy
import tf
from tf.transformations import euler_from_quaternion
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import TransformStamped, Transform, PointStamped

if __name__=="__main__":
    rospy.init_node("cut_contour")
    rospy.sleep(1)
    point = PointStamped()
    point.header.frame_id = "camera_color_optical_frame"
    point.point.x =  0.0671077857176
    point.point.y = 0.03886481308 
    point.point.z = 0.269
    transformer_listener = tf.TransformListener()
    transformer_listener.waitForTransform("link_6", "calibrated_frame", rospy.Time(),rospy.Duration(10))
    transform = transformer_listener.lookupTransform("link_6", "calibrated_frame", rospy.Time())
    print (transform[1])
    eurler = euler_from_quaternion(transform[1])
    # transformer_listener.transform
    # print(transform)
    print(eurler)
    # tf.TransformBroadcaster().sendTransform()
    # transformed_point = transformer_listener.transformPoint("base_link", point)
    # print(tf.TransformListener.allFramesAsString(transformer_listener))
    # print(transformed_point)
    
