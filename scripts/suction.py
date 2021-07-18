#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from cut_contour.robot_helpers import TransformServices, sample_from_func, MotionServices

signal_name = 'Suction_Cup'

class Suction():
    
    
    
    def __init__(self):
        self.done_pub = rospy.Publisher("/done", String, queue_size=1)
        
        rospy.Subscriber("/suction_xyz", PoseArray, self.suction_callback)
        
        self.tf_services = TransformServices()
        self.suction_ms = MotionServices('suction_cup')
        
        self.pre_place_goals = {'plastic box':'pre_front_left',
                            'hard disk box':'',
                            'fan box':'',
                            'CD ROM box':'pre_middle_right',
                            'motherboard box':'pre_middle_left',
                            'heat sink box':''}
        
        self.place_goals = {'plastic box':'front_left',
                            'hard disk box':'',
                            'fan box':'',
                            'CD ROM box':'middle_right',
                            'motherboard box':'middle_left',
                            'heat sink box':''}
        
    def suction_callback(self, suction_msg):
        #print("entered call_back = ", suction_msg)
        trans_suction_data = self.tf_services.transform_poses('base_link', 'calibrated_frame', suction_msg)
        
        
        trans_suction_data.poses[0].position.z += 0.05
        trans_suction_data.poses[0].orientation.x = 0.5
        trans_suction_data.poses[0].orientation.y = 0.5
        trans_suction_data.poses[0].orientation.z = 0.5
        trans_suction_data.poses[0].orientation.w = 0.5
        
        #print(trans_suction_data.poses[0])
        
        self.suction_ms.move_group.set_named_target('suction_pick')
        self.suction_ms.move_group.go()
        
        rospy.sleep(1)
        self.suction_ms.move_group.set_pose_reference_frame('base_link')
        self.suction_ms.move_group.set_pose_target(trans_suction_data.poses[0])
        self.suction_ms.move_group.go()
        
        
        rospy.sleep(1)
        touch_pose_data = trans_suction_data
        touch_pose_data.poses[0].position.z -= 0.1
        
        self.suction_ms.move_to_touch(touch_pose_data, 'y', force_thresh= 8, ref_frame= 'base_link')
        rospy.sleep(1)
        self.suction_ms.change_tool_status(signal_name, status=1)
        
        trans_suction_data.poses[0].position.z += 0.15
        
        self.suction_ms.move_straight(trans_suction_data)
        
        sorting_destination = rospy.get_param('sorting_destination')
        
        self.suction_ms.move_group.set_pose_reference_frame('base_link')
        self.suction_ms.move_group.set_named_target(self.pre_place_goals[sorting_destination])
        self.suction_ms.move_group.go()
        
        rospy.sleep(1)
        
        self.suction_ms.move_group.set_pose_reference_frame('base_link')
        self.suction_ms.move_group.set_named_target(self.place_goals[sorting_destination])
        self.suction_ms.move_group.go()
        rospy.sleep(1)
        self.suction_ms.change_tool_status(signal_name, status = 0)
        
        self.suction_ms.move_group.set_pose_reference_frame('base_link')
        self.suction_ms.move_group.set_named_target(self.pre_place_goals[sorting_destination])
        self.suction_ms.move_group.go()
        rospy.sleep(1)
        
        done_msg = String()
        done_msg.data = "Gripping Done"
        self.done_pub.publish(done_msg)
        
if __name__ == "__main__":

    rospy.init_node("suction")
    suction_object = Suction()
    rospy.spin()
