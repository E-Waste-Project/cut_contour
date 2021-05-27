#!/usr/bin/env python  
import rospy
import tf
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
    transformer_listener.waitForTransform("base_link", "camera_color_optical_frame", rospy.Time(),rospy.Duration(10))
    # transformer_listener.lookupTransform()
    
    transformed_point = transformer_listener.transformPoint("base_link", point)
    # print(tf.TransformListener.allFramesAsString(transformer_listener))
    print(transformed_point)
    
