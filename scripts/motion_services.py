#! /usr/bin/env python

from cut_contour.msg import MoveStraightAction, MoveStraightActionFeedback
import actionlib
import rospy
import roslib
roslib.load_manifest('cut_contour')


class MotionServer:
  def __init__(self):
    self.move_straight_server = actionlib.SimpleActionServer(
        'move_straight', MoveStraightAction, self.move_straight, False)
    self.move_straight_server.start()

  def move_straight(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    feedback = MoveStraightActionFeedback(curr_pose_idx=1)
    self.move_straight_server.publish_feedback(feedback)
    self.move_straight_server.set_succeeded(result=True, text="DONE!!")


if __name__ == '__main__':
  rospy.init_node('motion_server')
  server = MotionServer()
  rospy.spin()
