#!/usr/bin/env python
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseArray, Pose, TwistStamped
from sensor_msgs.msg import JointState
from math import pi
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
import rospy
import numpy as np


# def cb(msg):
#     joint_list = list(msg.position)
#     jacob  = np.array(group.get_jacobian_matrix(joint_list))
#     inv_jacob = np.linalg.inv(jacob)
#     # cart_vel = np.zeros((1, 6))
#     cart_vel = np.array([-0.01, 0, 0, 0, 0, -0.0])
#     joint_vel = np.matmul(inv_jacob, cart_vel)
#     print (joint_vel)
#     pub_msg = Float64MultiArray()
#     pub_msg.data = joint_vel
#     pub.publish(pub_msg)

rospy.init_node("go_to_camera_pose")
group = MoveGroupCommander("cutting_tool")
# rospy.Subscriber("/joint_states", JointState, cb)
# pub = rospy.Publisher("/egm/joint_group_velocity_controller/command", Float64MultiArray, queue_size=1)
# rospy.spin()
# rospy.sleep(1)
# group.set_joint_value_target([-0.0684868908897785, -0.11693841310234898, -0.2734241858258862, -0.020855601487006416, 1.92420219766421, 1.420389111068546])
# group.go()
output_pub = rospy.Publisher("/servo_server/delta_twist_cmds",TwistStamped,latch=False,queue_size=1)
rospy.sleep(1)
seq = 0
rate = rospy.Rate(1.0)

while not rospy.is_shutdown():
    message_output = TwistStamped()
    message_output.header.stamp =  rospy.Time.now()
    message_output.header.frame_id = 'base_link' 
    message_output.header.seq = seq
    seq += 1
    message_output.twist.linear.x = 0.001
    message_output.twist.linear.y = 0 
    message_output.twist.linear.z = 0.0

    output_pub.publish(message_output)
    rate.sleep()
rospy.sleep(2)
message_output = TwistStamped()
message_output.header.stamp =  rospy.Time.now()
message_output.header.frame_id = '' 
message_output.header.seq = seq
seq += 1
message_output.twist.linear.x = 0
message_output.twist.linear.y = 0 
message_output.twist.linear.z = 0.0

output_pub.publish(message_output)
rospy.sleep(2)