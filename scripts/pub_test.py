#!/usr/bin/env python  

import rospy
from geometry_msgs.msg import Pose

if __name__ == "__main__":
  rospy.init_node("pub_test")
  rospy.sleep(1)
  test_pose = Pose()
  test_pose.position.x = 0
  test_pose.position.y = 0
  test_pose.position.z = 0
  test_pose.orientation.w = 1
  test_pose.orientation.x = 0
  test_pose.orientation.y = 0
  test_pose.orientation.z = 0
  pub = rospy.Publisher("/pb_topic", Pose, queue_size=1)
  rospy.sleep(1)
  pub.publish(test_pose)
  rospy.spin()