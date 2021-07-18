#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from cut_contour.robot_helpers import TransformServices, sample_from_func, MotionServices, generate_spiral
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from copy import deepcopy
from math import pi

cutting_tool_signal_name = 'CuttingTool'
gripping_signal_name = 'Robot_Valve'


class Gripping():

    def __init__(self):
        self.done_pub = rospy.Publisher("/done", String, queue_size=1)

        rospy.Subscriber('/grip_xyz', PoseArray, self.grip_callback)

        self.tf_services = TransformServices()
        self.grip_ms = MotionServices('gripper')
        self.cut_ms = MotionServices('cutting_tool')

        self.pre_place_goals = {'plastic box': 'pre_front_left',
                                'hard disk box': '',
                                'fan box': '',
                                'CD ROM box': 'pre_middle_right',
                                'motherboard box': 'pre_middle_left',
                                'heat sink box': ''}

        self.place_goals = {'plastic box': 'front_left',
                            'hard disk box': '',
                            'fan box': '',
                            'CD ROM box': 'middle_right',
                            'motherboard box': 'middle_left',
                            'heat sink box': ''}

        self.cut_approach_dist = 0.05
        self.depth_of_cut = 0.0045
        self.cutting_offset = 0.01
        self.distance_between_grippers = 0.036
        self.gripping_offset = 0.1
        self.gripping_depth = 0.003

        #self.cutting_quat_base_link = [0.7, 0, 0.7, 0]
        self.cutting_quat_base_link = [0, 0.707, 0, 0.707]
        self.gripper_quat_base_link = [0.5, -0.5, 0.5, -0.5]

    def grip_callback(self, gripping_msg):
        # Get Desired Pose with respect to base frame
        trans_gripping_data = self.tf_services.transform_poses(
            'base_link', 'calibrated_frame', gripping_msg)
        trans_gripping_data.poses[0].position.z += self.cut_approach_dist
        trans_gripping_data.poses[0].orientation.x = self.cutting_quat_base_link[0]
        trans_gripping_data.poses[0].orientation.y = self.cutting_quat_base_link[1]
        trans_gripping_data.poses[0].orientation.z = self.cutting_quat_base_link[2]
        trans_gripping_data.poses[0].orientation.w = self.cutting_quat_base_link[3]

        # cut first hole
        cutting_position_1, cutting_position_2 = self.cut_gripping_hole(
            trans_gripping_data.poses[0])

        # Apply Distance Between two holes
        cutting_position_2.position.y += self.distance_between_grippers

        # cut second hole
        cutting_position_2, cutting_position_3 = self.cut_gripping_hole(
            cutting_position_2)

        # self.grip_ms.move_group.set_pose_reference_frame('base_link')
        # self.grip_ms.move_group.set_named_target('pre_middle_left')
        # self.grip_ms.move_group.go()

        # rospy.sleep(1)

        self.grip_ms.move_group.set_pose_reference_frame('base_link')
        self.grip_ms.move_group.set_named_target('gripper_pick')
        self.grip_ms.move_group.set_planning_time(0.1)
        self.grip_ms.move_group.set_num_planning_attempts(5)
        self.grip_ms.move_group.go()

        rospy.sleep(1)

        # Open Gripper
        self.grip_ms.change_tool_status(cutting_tool_signal_name, status=0)

        # Adjust Gripper Orientation and approach with offset
        cutting_position_1.position.z += self.gripping_offset
        cutting_position_1.orientation.x = self.gripper_quat_base_link[0]
        cutting_position_1.orientation.y = self.gripper_quat_base_link[1]
        cutting_position_1.orientation.z = self.gripper_quat_base_link[2]
        cutting_position_1.orientation.w = self.gripper_quat_base_link[3]

        self.grip_ms.move_group.set_pose_reference_frame('base_link')
        self.grip_ms.move_group.set_pose_target(cutting_position_1)
        self.grip_ms.move_group.go()

        rospy.sleep(1)

        cutting_position_1.position.z -= self.gripping_offset

        # Open Gripper
        self.grip_ms.change_tool_status(gripping_signal_name, status=0)

        # create Pose Array
        cutting_position_array = PoseArray()
        cutting_position_array.poses.append(cutting_position_1)
        cutting_position_array.poses[0].position.z -= self.gripping_depth

        # move Gripper Down
        self.grip_ms.move_to_touch(cutting_position_array,'y', force_thresh=1, ref_frame='base_link')

        rospy.sleep(1)

        # Close Gripper
        self.grip_ms.change_tool_status(gripping_signal_name, status=1)

        rospy.sleep(1)

        # Move Gripper Up
        cutting_position_array.poses[0].position.z += (self.gripping_offset + self.gripping_depth)
        self.grip_ms.move_straight(cutting_position_array)

        rospy.sleep(1)

        sorting_destination = rospy.get_param('sorting_destination')

        self.grip_ms.move_group.set_pose_reference_frame('base_link')
        self.grip_ms.move_group.set_named_target(
            self.pre_place_goals[sorting_destination])
        self.grip_ms.move_group.go()
        rospy.sleep(1)

        self.grip_ms.move_group.set_pose_reference_frame('base_link')
        self.grip_ms.move_group.set_named_target(
            self.place_goals[sorting_destination])
        self.grip_ms.move_group.go()
        rospy.sleep(1)

        # open Gripper
        self.grip_ms.change_tool_status(gripping_signal_name, status=0)

        self.grip_ms.move_group.set_pose_reference_frame('base_link')
        self.grip_ms.move_group.set_named_target(
            self.pre_place_goals[sorting_destination])
        self.grip_ms.move_group.go()
        rospy.sleep(1)

        done_msg = String()
        done_msg.data = "Gripping Done"
        self.done_pub.publish(done_msg)

    # Hole Cutting Sequence

    def cut_gripping_hole(self, approach_pose):
        
        self.flipping_ms.change_tool_status('Clamp_Off', status = 0)
        rospy.sleep(1)
        self.flipping_ms.change_tool_status('Clamp_On', status = 1)
        
        self.cut_ms.move_group.set_pose_reference_frame('base_link')
        self.cut_ms.move_group.set_named_target('cutting_start')
        self.cut_ms.move_group.set_planning_time(1)
        self.cut_ms.move_group.set_num_planning_attempts(5)
        self.cut_ms.move_group.go()
        
        rospy.sleep(1)
        
        print(approach_pose)
        
        self.cut_ms.move_group.set_pose_reference_frame('base_link')
        self.cut_ms.move_group.set_pose_target(approach_pose)
        self.cut_ms.move_group.set_planning_time(5)
        self.cut_ms.move_group.set_num_planning_attempts(25)
        self.cut_ms.move_group.go()

        rospy.sleep(1)

        approach_pose_array = PoseArray()
        approach_pose_array.poses.append(deepcopy(approach_pose))

        approach_pose_array.poses[0].position.z -= 0.1

        self.cut_ms.change_tool_status(cutting_tool_signal_name, status=0)
        self.cut_ms.move_to_touch(
            approach_pose_array, 'x', force_thresh=1, ref_frame='base_link')
        
        rospy.sleep(1)
        
        cutting_position = self.tf_services.lookup_transform(
            'base_link', 'milling_tool')
        cutting_position.position.z += self.cutting_offset
        cutting_position.orientation.x = self.cutting_quat_base_link[0]
        cutting_position.orientation.y = self.cutting_quat_base_link[1]
        cutting_position.orientation.z = self.cutting_quat_base_link[2]
        cutting_position.orientation.w = self.cutting_quat_base_link[3]
        
        print('cutting_position = ', cutting_position )
        
        # create Pose Array
        cutting_position_array = PoseArray()
        cutting_position_array.poses.append(deepcopy(cutting_position))
        self.cut_ms.move_straight(cutting_position_array)
        rospy.sleep(1)
        
        # self.cut_ms.move_group.set_pose_reference_frame('base_link')
        # self.cut_ms.move_group.set_named_target('cutting_start')
        # self.cut_ms.move_group.set_planning_time(1)
        # self.cut_ms.move_group.set_num_planning_attempts(5)
        # self.cut_ms.move_group.go()
        
        # rospy.sleep(1)
        
        # self.cut_ms.move_group.set_pose_reference_frame('base_link')
        # self.cut_ms.move_group.set_pose_target(approach_pose)
        # self.cut_ms.move_group.set_planning_time(5)
        # self.cut_ms.move_group.set_num_planning_attempts(25)
        # self.cut_ms.move_group.go()

        # rospy.sleep(1)
        
        self.cut_ms.change_tool_status(cutting_tool_signal_name, status=1)
        
        rospy.sleep(1)

        cutting_position_array.poses[0].position.z -= (self.cutting_offset + self.depth_of_cut)
        spiral_cutting_position_array = generate_spiral(X = 0.01, Y = 0.01, ref_pose = cutting_position_array.poses[0], ref_frame="base_link", step=1)
        self.cut_ms.move_straight(spiral_cutting_position_array)
        
        rospy.sleep(1)
        
        cutting_position_2 = deepcopy(cutting_position_array.poses[0])
        cutting_position_2.position.z += (self.cutting_offset + self.depth_of_cut)

        cutting_position_2_array = PoseArray()
        cutting_position_2_array.poses.append(deepcopy(cutting_position_2))
        self.cut_ms.move_straight(cutting_position_2_array)
        
        rospy.sleep(1)
        
        self.cut_ms.change_tool_status(cutting_tool_signal_name, status=0)
        
        rospy.sleep(1)
        
        return cutting_position_array.poses[0], cutting_position_2_array.poses[0]


if __name__ == "__main__":
    rospy.init_node("gripper")
    suction_object = Gripping()
    rospy.spin()
