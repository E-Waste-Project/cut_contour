#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from cut_contour.robot_helpers import TransformServices, sample_from_func, MotionServices


class Suction():
    
    signal_name = ''
    
    def __init__(self):
        self.done_pub = rospy.Publisher("/done", String, queue_size=1)
        
        rospy.Subscriber('/suction_xyz', PoseArray, self.suction_callback)
        
        self.tf_services = TransformServices()
        self.suction_ms = MotionServices('suction_cup')
        
        self.place_goals = {'plastic box':'',
                            'hard disk box':'',
                            'fan box':'',
                            'CD ROM box':'',
                            'motherboard box':'',
                            'heat sink box':''}
        
    def suction_callback(self, suction_msg):
        trans_suction_data = self.tf_services.transform_poses('base_link', 'calibrated_frame', suction_msg)
        
        
        trans_suction_data.poses[0].position.z += 0.02
        trans_suction_data.poses[0].orientation.x = 0
        trans_suction_data.poses[0].orientation.y = 0.707
        trans_suction_data.poses[0].orientation.z = 0
        trans_suction_data.poses[0].orientation.w = 0.707
        
        
        self.suction_ms.move_to_touch(trans_suction_data, 'z', force_thresh= 0.2, ref_frame= 'base_link')
        rospy.sleep(1)
        self.suction_ms.change_tool_status(signal_name, status=1)
        
        trans_suction_data.poses[0].position.z += 0.05
        
        self.suction_ms.move_straight(trans_suction_data)
        
        self.flipping_ms.move_group.set_named_target("flipping_pose")
        self.flipping_ms.move_group.go()
        
        sorting_destination = rospy.get_param('sorting destination')
        
        self.flipping_ms.move_group.set_named_target(self.place_goals[sorting_destination])
        self.flipping_ms.move_group.go()
        rospy.sleep(1)
        self.suction_ms.change_tool_status(signal_name, status = 0)
        
        done_msg = String()
        done_msg.data = "Gripping Done"
        self.done_pub.publish(done_msg)
