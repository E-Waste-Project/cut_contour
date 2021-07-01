#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String, Bool
from cut_contour.robot_helpers import MotionServices, TransformServices
from geometry_msgs.msg import PoseArray, Pose
from copy import deepcopy
from math import pi, atan2, fabs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_msgs.msg import Constraints


class CompetitionOperations():
    def __init__(self):
        rospy.Subscriber("/left_battery_topic", Pose, self.left_battery_cb)
        rospy.Subscriber("/right_battery_topic", Pose, self.right_battery_cb)
        rospy.Subscriber("/pb_topic", Pose, self.pb_cb)
        rospy.Subscriber("/cover_topic", Pose, self.cover_cb)
        rospy.Subscriber("/battery_hole_spiral_topic",
                         PoseArray, self.battery_hole_spiral_cb)
        rospy.Subscriber("/ethernet_empty_spiral_topic",
                         PoseArray, self.ethernet_empty_spiral_cb)
        rospy.Subscriber("/ethernet_topic", PoseArray, self.ethernet_cb)
        rospy.Subscriber("/key_topic", Pose, self.key_cb)
        rospy.Subscriber("/key_hole_spiral_topic", PoseArray, self.key_hole_spiral_cb)
        rospy.Subscriber("/board_state_topic", String, self.flip_cb)
        self.capture_pub = rospy.Publisher(
            "/capture_state", String, queue_size=1)
        self.done_pub = rospy.Publisher("/done", String, queue_size=1)
        self.battery_picker_ms = MotionServices(
            tool_group="gripper_battery_picker_group")
        self.battery_ms = MotionServices(tool_group="gripper_battery_group")
        self.pb_cover_ms = MotionServices(tool_group="gripper_pb_cover_group")
        self.camera_ms = MotionServices(tool_group="camera_group")
        self.ethernet_ms = MotionServices(tool_group="gripper_ethernet_group")
        self.key_ms = MotionServices(tool_group="gripper_key_group")
        self.tf_services = TransformServices()
        # self.x_comp_pb = -0.005
        # self.y_comp_pb = 0.004
        self.x_comp_pb = 0.0004
        self.y_comp_pb = 0.0015
        
        self.x_comp_key = 0.0
        self.y_comp_key = 0.007
        
        self.x_comp_key_hole = -0.0006
        self.y_comp_key_hole = 0.007     
        
        self.x_comp_cover = 0.0034
        self.y_comp_cover = 0.001        
        
        self.x_comp_battery = -0.0006
        self.y_comp_battery = 0.005
        
        self.left_comp = -0.004

        self.x_comp_battery_hole = 0.0064
        self.y_comp_battery_hole = 0.007
        
        self.x_comp_ethernet = 0.0004
        self.y_comp_ethernet = 0.0015
        
        self.x_comp_ethernet_hole = 0.004
        self.y_comp_ethernet_hole = 0.002
                        
        self.hole_pose = None
        self.first_detected_hole_pose = None
        self.flipped = False
        rospy.sleep(1)

    def flip_cb(self, msg):
        rospy.loginfo("Recieved Flipped Message")
        
        board_state = msg.data
        if board_state == "Flipped":
            self.flipped = True
            self.x_comp_pb *= 0
            self.y_comp_pb *= 0
            
            self.x_comp_key *= 0
            self.y_comp_key *= 0
            
            self.x_comp_key_hole *= 0
            self.y_comp_key_hole *= 0  
            
            self.x_comp_cover *= 0
            self.y_comp_cover *= 0       
            
            self.x_comp_battery *= 0
            self.y_comp_battery *= 0
            
            self.left_comp *= 0

            self.x_comp_battery_hole *= 0
            self.y_comp_battery_hole *= 0
            
            self.x_comp_ethernet *= 0
            self.y_comp_ethernet *= 0
            
            self.x_comp_ethernet_hole *= 0
            self.y_comp_ethernet_hole *= 0
        else:
            self.flipped = False
        rospy.sleep(3)
        
        print (self.flipped)
        # publish done msg
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)
        
    def pb_cb(self, msg):
        rospy.loginfo("Recieved Message")
        
        # Open Gripper
        self.battery_ms.change_tool_status(status=0, signal="Robothon_EE")
        # recieve pb center.
        pb_pose = msg
        print(pb_pose)

        # comp = comp if self.flipped else -comp
        # construct and move to pb pose within a safe distance.
        pose_array = PoseArray()
        pb_pose.position.y += self.y_comp_pb
        pb_pose.position.x += self.x_comp_pb
        q = [pb_pose.orientation.x, pb_pose.orientation.y,
             pb_pose.orientation.z, pb_pose.orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        angles[2] += pi/2 if self.flipped else -pi/2
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        pb_pose.orientation.x = q[0]
        pb_pose.orientation.y = q[1]
        pb_pose.orientation.z = q[2]
        pb_pose.orientation.w = q[3]
        pose_array.poses.append(pb_pose)
        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        pose_array.poses[0].position.z += 0.03
        q = [pose_array.poses[0].orientation.x, pose_array.poses[0].orientation.y,
        pose_array.poses[0].orientation.z, pose_array.poses[0].orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        angles[0] = pi
        angles[1] = 0
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        pose_array.poses[0].orientation.x = q[0]
        pose_array.poses[0].orientation.y = q[1]
        pose_array.poses[0].orientation.z = q[2]
        pose_array.poses[0].orientation.w = q[3]
        self.pb_cover_ms.move_group.clear_pose_targets()
        self.pb_cover_ms.move_group.set_pose_reference_frame("base_link")
        self.pb_cover_ms.move_group.set_pose_target(pose_array.poses[0])
        self.pb_cover_ms.move_group.go()
        # self.pb_cover_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)

        # move to push the button
        pb_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_pb_cover")
        pb_pose.position.z -= 0.08
        pose_array.poses[0] = pb_pose
        self.pb_cover_ms.move_to_touch(pose_array, axis='xy', force_thresh=5)
        rospy.loginfo("Pushed !!!!")

        # rospy.sleep(1) 

        # move up
        pb_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_pb_cover")
        pb_pose.position.z += 0.06
        pose_array.poses[0] = pb_pose
        self.pb_cover_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)
        rospy.loginfo("Moved !!!!")

        # rospy.sleep(1)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

    def cover_cb(self, msg):
        rospy.loginfo("Recieved Message")

        # recieve pb center.
        cover_pose = msg
        print(cover_pose)

        # construct and move to pb pose within a safe distance.
        pose_array = PoseArray()
        cover_pose.position.z -= 0.02
        cover_pose.position.y += self.y_comp_cover
        cover_pose.position.x += self.x_comp_cover
        q = [cover_pose.orientation.x, cover_pose.orientation.y,
             cover_pose.orientation.z, cover_pose.orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        angles[-1] = angles[-1] if self.flipped else angles[-1] + pi
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        cover_pose.orientation.x = q[0]
        cover_pose.orientation.y = q[1]
        cover_pose.orientation.z = q[2]
        cover_pose.orientation.w = q[3]
        pose_array.poses.append(cover_pose)
        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)

        q = [pose_array.poses[0].orientation.x, pose_array.poses[0].orientation.y,
             pose_array.poses[0].orientation.z, pose_array.poses[0].orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        print(angles)
        angles[0] = pi
        angles[1] = 0
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        pose_array.poses[0].orientation.x = q[0]
        pose_array.poses[0].orientation.y = q[1]
        pose_array.poses[0].orientation.z = q[2]
        pose_array.poses[0].orientation.w = q[3]
        self.pb_cover_ms.move_group.clear_pose_targets()
        self.pb_cover_ms.move_group.set_pose_reference_frame("base_link")
        self.pb_cover_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.pb_cover_ms.move_group.go()
        # self.pb_cover_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)
        if not result:
            rospy.sleep(3)
        
        # move to touch on the cover
        cover_pose = self.tf_services.lookup_transform("base_link", "gripper_pb_cover")
        cover_pose.position.z -= 0.1
        pose_array.poses[0] = cover_pose
        self.pb_cover_ms.move_to_touch(pose_array, axis='xy', force_thresh=20)
        rospy.loginfo("Touched !!!!")

        # open cover (move right in +x)
        self.tf_services.create_frame(
            "base_link", "gripper_pb_cover", "gripper_cover")
        remove_cover_pose = Pose()
        remove_cover_pose.position.x = 0.09
        remove_cover_pose.position.y = 0.0
        remove_cover_pose.position.z = 0.0
        remove_cover_pose.orientation.x = 0
        remove_cover_pose.orientation.y = 0
        remove_cover_pose.orientation.z = 0
        remove_cover_pose.orientation.w = 1
        pose_array.poses[0] = remove_cover_pose
        self.pb_cover_ms.move_straight(
            pose_array, vel_scale=1, acc_scale=1, ref_frame="gripper_cover")
        rospy.loginfo("Opened !!!!")

        # # move up flipped=False
        # cover_pose = self.tf_services.lookup_transform(not 
        #     "base_link", "gripper_pb_cover")
        # cover_pose.position.z += 0.1
        # pose_array.poses[0] = cover_pose
        # self.pb_cover_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)
        # rospy.loginfo("Moved !!!!")

        # rospy.sleep(1)

        # Return to Capture Pose
        capture_pose = self.tf_services.lookup_transform(
            "base_link", "capture_frame")
        self.camera_ms.move_group.clear_pose_targets()
        self.camera_ms.move_group.set_pose_reference_frame("base_link")
        self.camera_ms.move_group.set_pose_target(capture_pose)
        self.camera_ms.move_group.go()

        # rospy.sleep(1)

        # publish done msg
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

        # rospy.sleep(1)

        capture_msg = String()
        capture_msg.data = "Detect"
        self.capture_pub.publish(capture_msg)

    def battery_operation(self, battery_pose, loc="left"):

        battery_pose.position.y += self.y_comp_battery
        battery_pose.position.x += self.x_comp_battery

        # Open Gripper
        self.battery_ms.change_tool_status(status=0, signal="Robothon_EE")

        pose_array = PoseArray()
        pose_array.poses.append(battery_pose)
        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        pose_array.poses[0].position.z += 0.07
        q = [pose_array.poses[0].orientation.x, pose_array.poses[0].orientation.y,
             pose_array.poses[0].orientation.z, pose_array.poses[0].orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        print(angles)
        angles[0] = pi
        angles[1] = 0
        angles[2] = angles[2] if loc == "left" else angles[2] + pi
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        pose_array.poses[0].orientation.x = q[0]
        pose_array.poses[0].orientation.y = q[1]
        pose_array.poses[0].orientation.z = q[2]
        pose_array.poses[0].orientation.w = q[3]

        print("Battery Pose After Modification = ", pose_array.poses[0])

        # move to battery pose.
        self.battery_picker_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.battery_picker_ms.move_group.clear_pose_targets()
        self.battery_picker_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.battery_picker_ms.move_group.go()
        if not result:
            rospy.sleep(3)
        # self.battery_picker_ms.move_straight(
            # pose_array, vel_scale=1, acc_scale=1)

        print("Moved to Battery Pose")
        
        # rospy.sleep(1)

        # move to touch the battery axis using battery picker movegroup, while keeping a certain amount of force.
        pose_array.poses[0].position.z -= 0.2
        self.battery_picker_ms.move_to_touch(
            pose_array, axis='xy', force_thresh=10)

        print("Touched!!!!")

        # rospy.sleep(1)

        # move left until the fz equals zero.
        pose_array = self.battery_picker_ms.shift_pose_by(
            tf_services=self.tf_services, ref_frame="base_link", moving_frame="gripper_battery_picker",
            trans=[-0.04, 0, 0], new_frame="battery_fixed_frame")

        self.battery_picker_ms.move_to_touch(
            pose_array, axis='xy', force_thresh=8, ref_frame="battery_fixed_frame")

        print("Moved Left!!!!")

        # rospy.sleep(1)

        # move to touch in z direction.
        battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery_picker")
        battery_pose.position.z -= 0.021
        pose_array.poses[0] = battery_pose
        self.battery_picker_ms.move_to_touch(
            pose_array, axis='xy', force_thresh=9)
        print("Touched!!!")

        # rospy.sleep(1)

        # move to touch in right direction until certain force is achieved.
        right_dist = 0.01
        pose_array = self.battery_picker_ms.shift_pose_by(
            self.tf_services, "base_link", "gripper_battery_picker", trans=(right_dist, 0, 0), new_frame="battery_fixed_frame")
        self.battery_picker_ms.move_to_touch(
            pose_array, axis='z', force_thresh=8, ref_frame="battery_fixed_frame")
        print("Moved Right!!!!")

        # rospy.sleep(1)

        # move up by a certain amount.
        pose_array = self.battery_picker_ms.shift_pose_by(
            self.tf_services, "base_link", "gripper_battery_picker", trans=(0.003, 0, 0), new_frame="battery_fixed_frame")
        pose_array = self.tf_services.transform_poses(
            "base_link", pose_array.header.frame_id, pose_array)
        pose_array.poses[0].position.z += 0.03
        result = self.battery_picker_ms.move_straight(
            pose_array, vel_scale=0.1, acc_scale=0.1)
        if not result:
            rospy.sleep(3)
        print("Moved Up!!!!")

        # rospy.sleep(1)

        # move left
        pose_array = self.battery_picker_ms.shift_pose_by(
            self.tf_services, "base_link", "gripper_battery_picker", trans=(-0.035, 0, 0), new_frame="battery_fixed_frame")
        result = self.battery_picker_ms.move_straight(
            pose_array, ref_frame="battery_fixed_frame")
        if not result:
            rospy.sleep(3)
        print("Moved Left!!!!")
        print("DONE!!!!")

        # rospy.sleep(1)

        # move up by a certain amount.
        battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery_picker")
        battery_pose.position.z += 0.023
        pose_array.poses[0] = battery_pose
        result = self.battery_picker_ms.move_straight(
            pose_array, vel_scale=0.5, acc_scale=0.5)
        if not result:
            rospy.sleep(3)
        print("Moved Up!!!!")

        # rospy.sleep(1)

        # Get current position of gripper_battery_picker as it is the picking pose
        picking_pose = self.tf_services.lookup_transform(
            target_frame="base_link", source_frame="gripper_battery_picker")

        # Orient to the correct gripper orientation
        pose_array = self.battery_ms.shift_pose_by(
            self.tf_services, ref_frame="base_link", moving_frame="gripper_battery", rot=(0, 0, pi-0.01))
        pose_array = self.tf_services.transform_poses(
            "base_link", pose_array.header.frame_id, pose_array)
        self.battery_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.battery_ms.move_group.clear_pose_targets()
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, pose_array.header.frame_id, vel_scale=1, acc_scale=1)
        if not result:
            rospy.sleep(3)
        print("Oriented to backward gripper Orientation!!!!")

        # rospy.sleep(1)

        # go to battery picking pose.
        # Make the battery move_group go to the right poistioin.
        pose_array.poses[0].position = picking_pose.position
        gripper_battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        pose_array.poses[0].orientation = gripper_battery_pose.orientation
        self.battery_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)
        if not result:
            rospy.sleep(3)
        print("Moved the battery group!!!!")

        # rospy.sleep(1)

        # Orient the battery move_group to the battery orientation, and move it back a little.
        battery_angle = atan2(1.8, 4.8)
        pose_array = self.battery_ms.shift_pose_by(self.tf_services, "base_link", "gripper_battery", trans=(0, 0, 0),
                                                   rot=(0, battery_angle, 0))
        pose_array = self.tf_services.transform_poses(
            "base_link", pose_array.header.frame_id, pose_array)
        self.battery_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, "gripper_battery")
        if not result:
            rospy.sleep(3)
        print("Oriented to battery Orientation!!!!")

        # rospy.sleep(1)

        # Adjust position for gripping.
        adjust_x = -0.03
        adjust_y = -0.004
        pose_array = self.battery_ms.shift_pose_by(self.tf_services, ref_frame="base_link", moving_frame="gripper_battery",
                                                   new_frame="gripping_position", trans=(adjust_x, adjust_y, 0))
        pose_array = self.tf_services.transform_poses(
            "base_link", pose_array.header.frame_id, pose_array)
        self.battery_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, "gripper_battery")
        if not result:
            rospy.sleep(3)
        print("Adjust position for gripping!!!!")

        # Move to touch vertically.
        battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        battery_pose.position.z -= 0.05
        pose_array.poses = [battery_pose]
        self.battery_ms.move_to_touch(pose_array, axis='xy', force_thresh=5)
        print("Touched!!!")

        # rospy.sleep(1)

        # Grip
        self.battery_ms.change_tool_status(status=1, signal="Robothon_EE")

        rospy.sleep(1)

        # Move upwards.
        battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        battery_pose.position.z += 0.06
        pose_array.poses = [battery_pose]
        result = self.battery_ms.move_straight(
            pose_array, vel_scale=0.01, acc_scale=0.01)
        print("Moved Upwards!!!")
        if not result:
            rospy.sleep(3)

        # rospy.sleep(1)
        # Orient the battery move_group to make the battery vertical.
        # pose_array = self.battery_ms.shift_pose_by(
        #     self.tf_services, "base_link", "gripper_batpose_arraytery", rot=(0, -pi/2 + battery_angle, 0))
        # self.battery_ms.move_straight(
        #     pose_array, "gripper_battery", vel_scale=1, acc_scale=1)
        self.camera_ms.move_group.set_named_target("camera_pose")
        result = self.camera_ms.move_group.go()
        if not result:
            rospy.sleep(3)
        print("Oriented to battery Orientation to Vertical!!!!")
        
        rospy.sleep(1)
        
        target = "battery_pose_left" if self.flipped else "battery_pose_right"
        
        self.battery_ms.move_group.set_named_target(target)
        result = self.battery_ms.move_group.go()
        if not result:
            rospy.sleep(3)
        print("Oriented to correct starting pose, Ready for Spiral!!!!")

        # rospy.sleep(1)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)
        
    def left_battery_cb(self, msg):

        # recieve battery pose
        battery_pose = msg
        if not self.flipped:
            battery_pose.position.y += self.left_comp
        print("Recieved Left Battery Pose = ", battery_pose)
        self.battery_operation(battery_pose)

    def right_battery_cb(self, msg):

        # recieve battery pose
        battery_pose = msg
        if self.flipped:
            battery_pose.position.y += self.left_comp
        print("Recieved Right Battery Pose = ", battery_pose)
        self.battery_operation(battery_pose, loc="right")
        
    def battery_hole_spiral_cb(self, msg):
        pose_array = msg
        print("Recieved Battery Hole Pose")
        for pose in pose_array.poses:
            pose.position.x += self.x_comp_battery_hole
            pose.position.y += self.y_comp_battery_hole
        # if self.hole_pose is not None:
        #     shift_x = self.first_detected_hole_pose.position.x - self.hole_pose.position.x
        #     shift_y = self.first_detected_hole_pose.position.y - self.hole_pose.position.y
        #     for pose in pose_array.poses:
        #         pose.position.x -= shift_x
        #         pose.position.y -= shift_y
        #         # pose.position.x += shift_x
        #         # pose.position.y += shift_y
        # else:
        #     self.first_detected_hole_pose = pose_array.poses[0]
        spiral_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        gripper_battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        for pose in spiral_array.poses:
            pose.orientation = gripper_battery_pose.orientation

        # Move above hole with a safe distance
        pose_array = PoseArray()
        pose_array.poses.append(deepcopy(spiral_array.poses[0]))
        pose_array.poses[0].position.z = 0.25
        self.battery_ms.move_group.clear_pose_targets()
        # self.tf_services.create_frame("base_link", "gripper_battery", "gripper_battery_fixed")
        self.battery_ms.move_group.set_pose_reference_frame("base_link")
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.battery_ms.move_group.go()
        # pose_array = self.tf_services.transform_poses("gripper_battery_fixed", "base_link", pose_array)
        # step = 5
        # plan = None
        # while not rospy.is_shutdown():
        #     plan = self.battery_ms.move_group.plan(pose_array.poses[0])
        #     # print("plan points = ", plan.joint_trajectory.points)
        #     if not plan.joint_trajectory.points:
        #         q = [pose_array.poses[0].orientation.x, pose_array.poses[0].orientation.y,
        #              pose_array.poses[0].orientation.z, pose_array.poses[0].orientation.w]
        #         angles = euler_from_quaternion(q)
        #         angles = list(angles)
        #         angles[0] += step * pi/180.0
        #         if angles[0] > 2*pi:
        #             angles[0] -= 2*pi
        #         q = quaternion_from_euler(angles[0], angles[1], angles[2])
        #         pose_array.poses[0].orientation.x = q[0]
        #         pose_array.poses[0].orientation.y = q[1]
        #         pose_array.poses[0].orientation.z = q[2]
        #         pose_array.poses[0].orientation.w = q[3]
        #         continue
        #     else:
        #         break
        # result = self.battery_ms.move_group.execute(plan, wait=True)
        # self.battery_ms.move_straight(pose_array, vel_scale=0.1, acc_scale=0.1)
        print("Moved to Pose within safe distance")
        if not result:
            rospy.sleep(3)

        # rospy.sleep(1)
        
        # Move vertically down to touch
        pose = self.tf_services.lookup_transform("base_link", "gripper_battery")
        pose_array.header.frame_id = "base_link"
        pose_array.poses[0] = pose
        pose_array.poses[0].position.z = 0.13
        result = self.battery_ms.move_to_touch(pose_array, axis='x', force_thresh=3)
        print("Touched!!!!!")

        # rospy.sleep(1)
        

        if result != 3:
            print("Start Spiral!!!!")
            # Move spirally until sudden change in z force.4
            gripper_pose = self.tf_services.lookup_transform(
                "base_link", "gripper_battery")
            for pose in spiral_array.poses:
                pose.position.z = gripper_pose.position.z
                pose.orientation = gripper_pose.orientation
            self.battery_ms.move_to_touch(spiral_array, axis='x', force_thresh=2)

            # rospy.sleep(1)
        
        # Record Hole Position
        if self.hole_pose is None:
            self.hole_pose = self.tf_services.lookup_transform(
                "capture_frame", "gripper_battery")

        # Move downwards.
        battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        battery_pose.position.z -= 0.002
        pose_array.poses = [battery_pose]
        self.battery_ms.move_group.clear_pose_targets()
        self.battery_ms.move_group.set_pose_reference_frame("base_link")
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.battery_ms.move_group.go()
        # result = self.battery_ms.move_straight(pose_array, vel_scale=0.01, acc_scale=0.01)
        print("Moved Downwards!!!")
        if not result:
            rospy.sleep(3)

        # Open Gripper
        self.battery_ms.change_tool_status(status=0, signal="Robothon_EE")

        # rospy.sleep(1)
        
        # move upwards
        pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        pose_array.poses[0].position.z += 0.03
        self.battery_ms.move_straight(pose_array, vel_scale=0.1, acc_scale=0.1)
        
        # rospy.sleep(1)

        # close the gripper
        self.battery_ms.change_tool_status(status=1, signal="Robothon_EE")
        
        rospy.sleep(1)

        # move to touch the battery
        pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        pose_array.poses[0].position.z -= 0.1
        self.battery_ms.move_to_touch(pose_array, axis='x', force_thresh=5)
        
        # rospy.sleep(1)
        
        # move upwards
        pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        pose_array.poses[0].position.z += 0.03
        self.battery_ms.move_straight(pose_array, vel_scale=0.1, acc_scale=0.1)
        
        # rospy.sleep(1)

        # Return to Capture Pose
        capture_pose = self.tf_services.lookup_transform(
            "base_link", "capture_frame")
        pose_array.poses = [capture_pose]
        self.camera_ms.move_group.clear_pose_targets()
        self.camera_ms.move_group.set_pose_reference_frame("base_link")
        self.camera_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.camera_ms.move_group.go()
        if not result:
            rospy.sleep(3)

        rospy.sleep(1)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

    def ethernet_cb(self, msg):
        pose_array = msg
        print("Received Ethernet Pose msg")

        # close the gripper
        self.battery_ms.change_tool_status("Robothon_EE", status=1)

        for pose in pose_array.poses:
            pose.position.x += self.x_comp_ethernet
            pose.position.y += self.y_comp_ethernet

        # pose_array.poses[1].position.x += 0.005

        # go to ethernet empty pose
        pose_array = self.tf_services.transform_poses("base_link", "capture_frame", pose_array)
        pose_array.poses[0].position.z += 0.1
        q = [pose_array.poses[0].orientation.x, pose_array.poses[0].orientation.y,
             pose_array.poses[0].orientation.z, pose_array.poses[0].orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        print(angles)
        angles[0] = pi
        angles[1] = 0
        angles[2] = angles[2] + pi if self.flipped else angles[2]
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        for pose in pose_array.poses:
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

        pose_array.poses[1].orientation = pose_array.poses[0].orientation
        self.ethernet_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.ethernet_ms.move_group.set_pose_target(pose_array.poses[0])
        self.ethernet_ms.move_group.go()

        # rospy.sleep(1)

        # Move to touch to know the actual depth
        ethernet_array = PoseArray()
        ethernet_array.poses.append(deepcopy(pose_array.poses[0]))
        ethernet_array.poses[0].position.z -= 0.15
        self.ethernet_ms.move_to_touch(
            ethernet_array, axis='xy', force_thresh=2)

        # rospy.sleep(1)

        # Move up a bit.
        ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")

        # adjust all poses depth to have the actual measured depth
        for pose in pose_array.poses:
            pose.position.z = ethernet_pose.position.z
        ethernet_pose.position.z += 0.002
        ethernet_array.poses = [ethernet_pose]
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=1, acc_scale=1)
        print("Moved Upwards!!!")
        if not result:
            rospy.sleep(3)

        # rospy.sleep(1)

        # approach the ethernet
        ethernet_array = self.ethernet_ms.shift_pose_by(self.tf_services, "base_link", "gripper_ethernet", "gripper_ethernet_fixed_1", trans=(0.2, 0, 0))
        result = self.ethernet_ms.move_to_touch(
            ethernet_array, axis='x', vel_scale=0.01, acc_scale=0.01, ref_frame=ethernet_array.header.frame_id, force_thresh=3)
        print("Approached Ethernet Cable!!!")
        if not result:
            rospy.sleep(3)

        rospy.sleep(1)
        
        # open the gripper
        self.battery_ms.change_tool_status("Robothon_EE", status=0)

        rospy.sleep(1)

        # approach the ethernet
        ethernet_array = self.ethernet_ms.shift_pose_by(self.tf_services, "base_link", "gripper_ethernet", "gripper_ethernet_fixed_2", trans=(0.015, 0, 0))
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=0.01, acc_scale=0.01, ref_frame=ethernet_array.header.frame_id)
        print("Approached Ethernet Cable!!!")
        if not result:
            rospy.sleep(3)

        rospy.sleep(1)        
        

        # close the gripper
        self.ethernet_ms.change_tool_status("Robothon_EE", status=1)

        rospy.sleep(1)

        # move upwards
        ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        ethernet_pose.position.z += 0.05
        ethernet_array = PoseArray()
        ethernet_array.poses.append(ethernet_pose)
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=0.01, acc_scale=0.01)
        print("Moved Upwards!!!")
        if not result:
            rospy.sleep(3)

        # rospy.sleep(1)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

    def ethernet_empty_spiral_cb(self, msg):
        pose_array = msg
        print("Recieved ethernet Pose")
        comp = 0.003
        comp = comp if self.flipped else comp * -1
        for pose in pose_array.poses:
            pose.position.x += self.x_comp_ethernet_hole
            pose.position.y += self.y_comp_ethernet_hole

        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        gripper_ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        for pose in pose_array.poses:
            pose.orientation = gripper_ethernet_pose.orientation

        # rospy.sleep(1)

        # move to ethernet empty pose
        ethernet_array = PoseArray()
        ethernet_array.poses.append(deepcopy(pose_array.poses[0]))
        ethernet_array.poses[0].position.z += 0.03
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=0.1, acc_scale=0.1)
        print("Moved To Etherned Empty!!!")
        if not result:
            rospy.sleep(3)

        # rospy.sleep(1)
        
        # move to touch
        ethernet_array = PoseArray()
        ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        ethernet_array.poses.append(ethernet_pose)
        ethernet_array.poses[0].position.z -= 0.05
        self.ethernet_ms.move_to_touch(
            ethernet_array, axis='xy', force_thresh=3)
# 
        rospy.sleep(1)

        gripper_ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        init_z = gripper_ethernet_pose.position.z
        for pose in pose_array.poses:
            pose.position.z = init_z
        
        # check if ethernet has been inserted
        print("Check Insertion!!!!")
        inserted = True
        check_dist = 0.002
        for axis, t in [('x', (check_dist, 0, 0)), ('x', (-check_dist, 0, 0)),  ('y', (0, check_dist, 0)), ('y', (0, -check_dist, 0))]:
            ethernet_array = self.ethernet_ms.shift_pose_by(self.tf_services, ref_frame="base_link", moving_frame="gripper_ethernet",
                                        new_frame="touch_frame", trans=t)
            ethernet_array = self.tf_services.transform_poses("base_link", "touch_frame", ethernet_array)
            result = self.ethernet_ms.move_to_touch(ethernet_array, axis=axis, force_thresh=2, ref_frame="base_link")
            if result == 3:
                inserted = False
                break
    
        print("Start Spiral!!!!")
        spiral_pose_idx = 0
        spiral_sz = len(pose_array.poses)
        while fabs(gripper_ethernet_pose.position.z - init_z) <= 0.003 and not inserted and spiral_pose_idx < spiral_sz:
            
            # move above next spiral position
            spiral_array = PoseArray()
            spiral_array.poses.append(deepcopy(pose_array.poses[spiral_pose_idx]))
            spiral_array.poses[0].position.z += 0.002
            self.ethernet_ms.move_straight(spiral_array, vel_scale=1, acc_scale=1)
            
            
            # move to touch at next spiral position
            gripper_ethernet_pose = self.tf_services.lookup_transform(
                "base_link", "gripper_ethernet")
            gripper_ethernet_pose.position.z -= 0.05
            ethernet_array = PoseArray()
            ethernet_array.poses.append(gripper_ethernet_pose)
            self.ethernet_ms.move_to_touch(
                ethernet_array, axis='xy', force_thresh=3)


            gripper_ethernet_pose = self.tf_services.lookup_transform(
                "base_link", "gripper_ethernet")
            spiral_pose_idx += 1

        if spiral_pose_idx >= spiral_sz:
            print("couldn't insert the Ethernet")
            return
        # rospy.sleep(1)

        # Move downwards.
        gripper_ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        ethernet_array = PoseArray()
        ethernet_array.poses.append(gripper_ethernet_pose)
        ethernet_array.poses[0].position.z -= 0.05
        result = self.ethernet_ms.move_to_touch(
            ethernet_array, vel_scale=0.01, acc_scale=0.01, force_thresh=20, axis='xy')
        print("Ethernet inserted!!!")
        if not result:
            rospy.sleep(3)

        # rospy.sleep(1)

        # Open Gripper
        self.ethernet_ms.change_tool_status(status=0, signal="Robothon_EE")
        rospy.sleep(1)

        # Move Upwards.
        gripper_ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        ethernet_array = PoseArray()
        ethernet_array.poses.append(gripper_ethernet_pose)
        ethernet_array.poses[0].position.z += 0.01
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=0.01, acc_scale=0.01)
        print("Ethernet inserted!!!")
        if not result:
            rospy.sleep(3)
        # rospy.sleep(1)
        
        # Close the gripper
        self.ethernet_ms.change_tool_status(status=1, signal="Robothon_EE")
        rospy.sleep(1)
        
        # Move Downwards To Touch.
        gripper_ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        ethernet_array = PoseArray()
        ethernet_array.poses.append(gripper_ethernet_pose)
        ethernet_array.poses[0].position.z -= 0.02
        result = self.ethernet_ms.move_to_touch(
            ethernet_array, vel_scale=0.01, acc_scale=0.01, force_thresh=4, axis='xy')
        print("Ethernet inserted!!!")
        if not result:
            rospy.sleep(3)
        # rospy.sleep(1)
        
        # Open the gripper
        self.ethernet_ms.change_tool_status(status=0, signal="Robothon_EE")
        rospy.sleep(1)
        
        # Move Upwards.
        gripper_ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        ethernet_array = PoseArray()
        ethernet_array.poses.append(gripper_ethernet_pose)
        ethernet_array.poses[0].position.z += 0.03
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=0.01, acc_scale=0.01)
        print("Ethernet inserted!!!")
        if not result:
            rospy.sleep(3)
        # rospy.sleep(1)
        
        # Return to Capture Pose
        capture_pose = self.tf_services.lookup_transform(
            "base_link", "capture_frame")
        self.camera_ms.move_group.clear_pose_targets()
        self.camera_ms.move_group.set_pose_reference_frame("base_link")
        self.camera_ms.move_group.set_pose_target(capture_pose)
        result = self.camera_ms.move_group.go()
        if not result:
            rospy.sleep(3)
        
        # rospy.sleep(3)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

    def key_cb(self, msg):
        key_pose = msg
        key_pose.position.x += self.x_comp_key
        key_pose.position.y += self.y_comp_key


        # transform key from capture frame to base link
        pose_array = PoseArray()
        pose_array.poses.append(deepcopy(key_pose))
        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)

        q = [pose_array.poses[0].orientation.x, pose_array.poses[0].orientation.y,
             pose_array.poses[0].orientation.z, pose_array.poses[0].orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        print(angles)
        angles[0] = pi
        angles[1] = 0
        angles[2] = angles[2] if self.flipped else angles[2] + pi
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        pose_array.poses[0].orientation.x = q[0]
        pose_array.poses[0].orientation.y = q[1]
        pose_array.poses[0].orientation.z = q[2]
        pose_array.poses[0].orientation.w = q[3]

        # Key Array
        key_array = PoseArray()
        key_array = deepcopy(pose_array)

        # open the gripper
        self.key_ms.change_tool_status("Robothon_EE", status=0)

        # go to key pose
        pose_array = PoseArray()
        pose_array = deepcopy(key_array)
        pose_array.poses[0].position.z += 0.03
        self.key_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.key_ms.move_group.set_pose_target(pose_array.poses[0])
        self.key_ms.move_group.go()
        print("Moved To Key!!!")

        # rospy.sleep(1)

        # approach the key
        pose_array = PoseArray()
        pose_array = deepcopy(key_array)
        pose_array.poses[0].position.z -= 0.05
        self.key_ms.move_to_touch(
            pose_array, vel_scale=0.01, acc_scale=0.01, axis='xy', force_thresh=3)
        print("Key Reached!!!")

        # rospy.sleep(1)

        # move upwards
        gripper_key_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_key")
        pose_array = PoseArray()
        pose_array.poses.append(gripper_key_pose)

        pose_array.poses[0].position.z += 0.005
        result = self.key_ms.move_straight(
            pose_array, vel_scale=0.01, acc_scale=0.01)
        print("Ready to Pick the Key!!!")
        if not result:
            rospy.sleep(3)

        # rospy.sleep(1)

        # close the gripper
        self.key_ms.change_tool_status("Robothon_EE", status=1)

        # rospy.sleep(1)

        # move upwards
        pose_array.poses[0].position.z += 0.03
        result = self.key_ms.move_straight(
            pose_array, vel_scale=0.01, acc_scale=0.01)
        print("Key Picked!!!")
        if not result:
            rospy.sleep(3)

        # rospy.sleep(1)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)
    

    def key_hole_spiral_cb(self, msg):
        # go to the key hole pose
        key_spiral_poses = msg
        for pose in key_spiral_poses.poses:
            pose.position.x += self.x_comp_key_hole
            pose.position.y += self.y_comp_key_hole
        key_hole_pose = key_spiral_poses.poses[0]
        # transform key from capture frame to base link
        # pose_array = PoseArray()
        # pose_array.poses.append(deepcopy(key_hole_pose))
          
        key_spiral_poses = self.tf_services.transform_poses(
            "base_link", "capture_frame", key_spiral_poses)

        k_pose = key_spiral_poses.poses[0]
        q = [k_pose.orientation.x, k_pose.orientation.y,
             k_pose.orientation.z, k_pose.orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        angles[0] = pi
        angles[1] = 0
        angles[2] = angles[2] + pi/2 if self.flipped else angles[2] - pi/2
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        for pose in key_spiral_poses.poses:
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

        # Key Array[0
        key_hole_array = PoseArray()
        key_hole_array = deepcopy(key_spiral_poses)
        pose_array = PoseArray()
        pose_array.poses.append(deepcopy(key_hole_array.poses[0]))
        pose_array.poses[0].position.z += 0.06
        pose_array.header.frame_id = "base_link"
        self.key_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.key_ms.move_group.set_pose_target(pose_array.poses[0])
        self.key_ms.move_group.go()
        print("Went to Key hole Pose")
        
        # approach the key hole
        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link"
        pose_array.poses.append(deepcopy(key_hole_array.poses[0]))
        pose_array.poses[0].position.z = 0.115
        result = self.key_ms.move_to_touch(
            pose_array, vel_scale=0.01, acc_scale=0.01, axis='xy', force_thresh=8)
        arm = self.key_ms.current_arm['y']
        print("Result = ", result)
        print("Touched Key Hole Surface!!!")

        # rospy.sleep(1)
        
        current_pose = self.tf_services.lookup_transform("base_link", "gripper_key")
        for pose in key_spiral_poses.poses:
            pose.position.z = current_pose.position.z
        
        key_spiral_poses.header.frame_id = "gripper_key"
        if result != 3:
            print("Start Hole Search!!!!")
            self.key_ms.hole_search(self.tf_services, init_z=current_pose.position.z, pose_array=key_spiral_poses,
                                    z_thresh=0.01, z_upper=0.003, z_lower=-0.05,
                                    force_thresh=8, axis='xy', ref_frame="base_link",
                                    vel_scale=0.01, acc_scale=0.01)
            # rospy.sleep(1)        
         
        
        
        print("Start Inserting The Key")        
        # move to touch in +z wrt the insertion frame
        pose_array = self.key_ms.shift_pose_by(
            tf_services=self.tf_services, ref_frame="base_link", moving_frame="gripper_key",
            trans=[0.0, 0.0, 0.05], new_frame="gripper_hole_insertion_frame")

        self.key_ms.move_to_touch(
            pose_array, axis='xy', force_thresh=10, ref_frame=pose_array.header.frame_id)
        
        key_center_correction = 0.002
        print("arm = ", arm)
        if arm < -0.15 or arm > 0.15:
            if arm < 0:
                key_center_correction *= -1
            # Open the gripper
            self.key_ms.change_tool_status("Robothon_EE", status=0)
            rospy.sleep(1)
            # correct position
            pose_array = self.key_ms.shift_pose_by(self.tf_services, "base_link", "gripper_key", trans=(-0.002, 0, 0))
            result = self.key_ms.move_straight(pose_array, "new_frame", vel_scale = 1, acc_scale = 1)
            if not result:
                rospy.sleep(3)
            # rospy.sleep(1)
            # close the gripper
            self.key_ms.change_tool_status("Robothon_EE", status=1)
            rospy.sleep(1)
        
        # Open the gripper
        self.key_ms.change_tool_status("Robothon_EE", status=0)
        rospy.sleep(1)
        # move_up
        pose_array = self.key_ms.shift_pose_by(self.tf_services, "base_link", "gripper_key", trans=(0, 0, -0.002))
        result = self.key_ms.move_straight(pose_array, "new_frame", vel_scale = 1, acc_scale = 1)
        if not result:
            rospy.sleep(3)
        # rospy.sleep(1)
        # close the gripper
        self.key_ms.change_tool_status("Robothon_EE", status=1)
        rospy.sleep(1)
        
        # rotate key
        pose_array = self.key_ms.shift_pose_by(self.tf_services, "base_link", "gripper_key", rot=(0, 0, 48 * pi / 180.0))
        result = self.key_ms.move_straight(pose_array, "new_frame", vel_scale=0.1, acc_scale=0.1)    
        if not result:
            rospy.sleep(3)
        rospy.sleep(1)
        
        # rotate key
        pose_array = self.key_ms.shift_pose_by(self.tf_services, "base_link", "gripper_key", rot=(0, 0, -48 * pi / 180.0))
        result = self.key_ms.move_straight(pose_array, "new_frame", vel_scale=0.1, acc_scale=0.1)
        if not result:
            rospy.sleep(3)
        
        # rospy.sleep(1)
        
        # Open the gripper
        self.key_ms.change_tool_status("Robothon_EE", status=0)
        
        rospy.sleep(1)
        
        # Move Up
        key_pose = self.tf_services.lookup_transform("base_link", "gripper_key")
        key_pose.position.z += 0.03
        pose_array = PoseArray()
        pose_array.poses.append(key_pose)
        result = self.key_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)
        if not result:
            rospy.sleep(3)
        print("Moved Up!!!")
        
        # Return to Capture Pose
        capture_pose = self.tf_services.lookup_transform(
            "base_link", "capture_frame")
        self.camera_ms.move_group.clear_pose_targets()
        self.camera_ms.move_group.set_pose_reference_frame("base_link")
        self.camera_ms.move_group.set_pose_target(capture_pose)
        result = self.camera_ms.move_group.go()
        if not result:
            rospy.sleep(3)
        # rospy.sleep(3)
                
        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)   


if __name__ == "__main__":
    rospy.init_node("competition_operations")
    rospy.sleep(1)
    comp_op = CompetitionOperations()
    rospy.spin()
