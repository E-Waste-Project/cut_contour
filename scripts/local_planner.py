#!/usr/bin/env python
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import rospy
from math import pi

rospy.init_node("local_planner")

group = MoveGroupCommander("cutting_tool")
group.set_joint_value_target([0, 0.5, 0, 0, 0, 0])
group.go(wait=True)
# rospy.sleep(1)
# group.set_max_velocity_scaling_factor(1)
# current_state = group.get_current_state()
# current_state.joint_state = rospy.wait_for_message("/joint_states", JointState)
# group.set_start_state(current_state)
# group.go(wait=True)
