#!/usr/bin/env python
import rospy
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import Constraints, PositionConstraint

rospy.init_node("test")
group = MoveGroupCommander("camera_group")
constraints = Constraints()
# PositionConstraint().target_point_offset.
# constraints.position_constraints.
group.set_path_constraints()
