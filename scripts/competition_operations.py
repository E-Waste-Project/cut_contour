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
        rospy.Subscriber("/battery_hole_topic", Pose, self.battery_hole_cb)
        rospy.Subscriber("/battery_hole_spiral_topic",
                         PoseArray, self.battery_hole_spiral_cb)
        rospy.Subscriber("/ethernet_empty_spiral_topic",
                         PoseArray, self.ethernet_empty_spiral_cb)
        rospy.Subscriber("/ethernet_topic", PoseArray, self.ethernet_cb)
        rospy.Subscriber("/key_topic", Pose, self.key_cb)
        rospy.Subscriber("/flipped", Bool, self.flip_cb)
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
        self.x_comp = -0.005
        self.y_comp = 0.004
        self.hole_pose = None
        self.first_detected_hole_pose = None
        self.flipped = False

    def flip_cb(self, msg):
        self.flipped = msg.data

    def pb_cb(self, msg):
        rospy.loginfo("Recieved Message")

        # recieve pb center.
        pb_pose = msg
        print(pb_pose)

        # construct and move to pb pose within a safe distance.
        pose_array = PoseArray()
        pb_pose.position.y += self.y_comp
        pb_pose.position.x += self.x_comp
        q = [pb_pose.orientation.x, pb_pose.orientation.y,
             pb_pose.orientation.z, pb_pose.orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        angles[2] -= pi/2
        q = quaternion_from_euler(angles[0], angles[1], angles[2])
        pb_pose.orientation.x = q[0]
        pb_pose.orientation.y = q[1]
        pb_pose.orientation.z = q[2]
        pb_pose.orientation.w = q[3]
        pose_array.poses.append(pb_pose)
        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        pose_array.poses[0].position.z += 0.05
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
        pb_pose.position.z -= 0.1
        pose_array.poses[0] = pb_pose
        self.pb_cover_ms.move_to_touch(pose_array, axis='xy', force_thresh=5)
        rospy.loginfo("Pushed !!!!")

        rospy.sleep(5)

        # move up
        pb_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_pb_cover")
        pb_pose.position.z += 0.1
        pose_array.poses[0] = pb_pose
        self.pb_cover_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)
        rospy.loginfo("Moved !!!!")

        rospy.sleep(1)

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
        cover_pose.position.y += self.y_comp
        cover_pose.position.x += self.x_comp + 0.004
        q = [cover_pose.orientation.x, cover_pose.orientation.y,
             cover_pose.orientation.z, cover_pose.orientation.w]
        angles = euler_from_quaternion(q)
        angles = list(angles)
        angles[-1] += pi
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
        self.pb_cover_ms.move_group.go()
        # self.pb_cover_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)

        # move to touch on the cover
        cover_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_pb_cover")
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

        # move up
        cover_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_pb_cover")
        cover_pose.position.z += 0.1
        pose_array.poses[0] = cover_pose
        self.pb_cover_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)
        rospy.loginfo("Moved !!!!")

        rospy.sleep(1)

        # Return to Capture Pose
        capture_pose = self.tf_services.lookup_transform(
            "base_link", "capture_frame")
        self.camera_ms.move_group.clear_pose_targets()
        self.camera_ms.move_group.set_pose_reference_frame("base_link")
        self.camera_ms.move_group.set_pose_target(capture_pose)
        self.camera_ms.move_group.go()

        rospy.sleep(1)

        # publish done msg
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

        rospy.sleep(1)

        capture_msg = String()
        capture_msg.data = "Detect"
        self.capture_pub.publish(capture_msg)

    def battery_operation(self, battery_pose, loc="left"):

        battery_pose.position.y += self.y_comp
        battery_pose.position.x += self.x_comp

        # Open Gripper
        self.battery_ms.change_tool_status(status=0, signal="Robothon_EE")

        pose_array = PoseArray()
        pose_array.poses.append(battery_pose)
        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        pose_array.poses[0].position.z += 0.02
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
        self.battery_picker_ms.move_group.go()
        # self.battery_picker_ms.move_straight(
            # pose_array, vel_scale=1, acc_scale=1)

        print("Moved to Battery Pose")

        # move to touch the battery axis using battery picker movegroup, while keeping a certain amount of force.
        pose_array.poses[0].position.z -= 0.07
        self.battery_picker_ms.move_to_touch(
            pose_array, axis='xy', force_thresh=10)

        print("Touched!!!!")

        rospy.sleep(1)

        # move left until the fz equals zero.
        pose_array = self.battery_picker_ms.shift_pose_by(
            tf_services=self.tf_services, ref_frame="base_link", moving_frame="gripper_battery_picker",
            trans=[-0.04, 0, 0], new_frame="battery_fixed_frame")

        self.battery_picker_ms.move_to_touch(
            pose_array, axis='xy', force_thresh=7, ref_frame="battery_fixed_frame")

        print("Moved Left!!!!")

        rospy.sleep(1)

        # move to touch in z direction.
        battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery_picker")
        battery_pose.position.z -= 0.02
        pose_array.poses[0] = battery_pose
        self.battery_picker_ms.move_to_touch(
            pose_array, axis='xy', force_thresh=7)
        print("Touched!!!")

        rospy.sleep(1)

        # move to touch in right direction until certain force is achieved.
        right_dist = 0.01
        pose_array = self.battery_picker_ms.shift_pose_by(
            self.tf_services, "base_link", "gripper_battery_picker", trans=(right_dist, 0, 0), new_frame="battery_fixed_frame")
        self.battery_picker_ms.move_to_touch(
            pose_array, axis='z', force_thresh=7, ref_frame="battery_fixed_frame")
        print("Moved Right!!!!")

        rospy.sleep(1)

        # move up by a certain amount.
        pose_array = self.battery_picker_ms.shift_pose_by(
            self.tf_services, "base_link", "gripper_battery_picker", trans=(0.005, 0, 0), new_frame="battery_fixed_frame")
        pose_array = self.tf_services.transform_poses(
            "base_link", "gripper_battery_picker", pose_array)
        pose_array.poses[0].position.z += 0.02
        self.battery_picker_ms.move_straight(
            pose_array, vel_scale=0.5, acc_scale=0.5)
        print("Moved Up!!!!")

        rospy.sleep(1)

        # move left
        pose_array = self.battery_picker_ms.shift_pose_by(
            self.tf_services, "base_link", "gripper_battery_picker", trans=(-0.035, 0, 0), new_frame="battery_fixed_frame")
        self.battery_picker_ms.move_straight(
            pose_array, ref_frame="battery_fixed_frame")
        print("Moved Left!!!!")
        print("DONE!!!!")

        rospy.sleep(1)

        # move up by a certain amount.
        battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery_picker")
        battery_pose.position.z += 0.02
        pose_array.poses[0] = battery_pose
        self.battery_picker_ms.move_straight(
            pose_array, vel_scale=0.5, acc_scale=0.5)
        print("Moved Up!!!!")

        rospy.sleep(1)

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
        self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, pose_array.header.frame_id, vel_scale=1, acc_scale=1)
        print("Oriented to backward gripper Orientation!!!!")

        rospy.sleep(1)

        # go to battery picking pose.
        # Make the battery move_group go to the right poistioin.
        pose_array.poses[0].position = picking_pose.position
        gripper_battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        pose_array.poses[0].orientation = gripper_battery_pose.orientation
        self.battery_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)
        print("Moved the battery group!!!!")

        rospy.sleep(1)

        # Orient the battery move_group to the battery orientation, and move it back a little.
        battery_angle = atan2(1.8, 4.8)
        pose_array = self.battery_ms.shift_pose_by(self.tf_services, "base_link", "gripper_battery", trans=(0, 0, 0),
                                                   rot=(0, battery_angle, 0))
        pose_array = self.tf_services.transform_poses(
            "base_link", pose_array.header.frame_id, pose_array)
        self.battery_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, "gripper_battery")
        print("Oriented to battery Orientation!!!!")

        rospy.sleep(1)

        # Adjust position for gripping.
        adjust_x = -0.032
        adjust_y = -0.004
        pose_array = self.battery_ms.shift_pose_by(self.tf_services, ref_frame="base_link", moving_frame="gripper_battery",
                                                   new_frame="gripping_position", trans=(adjust_x, adjust_y, 0))
        pose_array = self.tf_services.transform_poses(
            "base_link", pose_array.header.frame_id, pose_array)
        self.battery_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, "gripper_battery")
        print("Adjust position for gripping!!!!")

        # Move to touch vertically.
        battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        battery_pose.position.z -= 0.03
        pose_array.poses = [battery_pose]
        self.battery_ms.move_to_touch(pose_array, axis='xy', force_thresh=2)
        print("Touched!!!")

        rospy.sleep(1)

        # Grip
        self.battery_ms.change_tool_status(status=1, signal="Robothon_EE")

        rospy.sleep(1)

        # Move upwards.ethernet_empty_spiral_cb
        battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        battery_pose.position.z += 0.05
        pose_array.poses = [battery_pose]
        result = self.battery_ms.move_straight(
            pose_array, vel_scale=0.01, acc_scale=0.01)
        print("Moved Upwards!!!")
        if not result:
            rospy.sleep(5)

        # Orient the battery move_group to make the battery vertical.
        # pose_array = self.battery_ms.shift_pose_by(
        #     self.tf_services, "base_link", "gripper_batpose_arraytery", rot=(0, -pi/2 + battery_angle, 0))
        # self.battery_ms.move_straight(
        #     pose_array, "gripper_battery", vel_scale=1, acc_scale=1)
        self.battery_ms.move_group.set_named_target("battery_pose_back")
        self.battery_ms.move_group.go()
        print("Oriented to battery Orientation to Vertical!!!!")

        rospy.sleep(5)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

    def left_battery_cb(self, msg):

        # recieve battery pose
        battery_pose = msg
        print("Recieved Left Battery Pose = ", battery_pose)

        self.battery_operation(battery_pose)

    def right_battery_cb(self, msg):

        # recieve battery pose
        battery_pose = msg
        print("Recieved Right Battery Pose = ", battery_pose)

        self.battery_operation(battery_pose, loc="right")

    def battery_hole_spiral_cb(self, msg):
        pose_array = msg
        print("Recieved Battery Hole Pose")
        for pose in pose_array.poses:
            pose.position.x += self.x_comp
            pose.position.y += self.y_comp
        if self.hole_pose is not None:
            shift_x = self.first_detected_hole_pose.position.x - self.hole_pose.position.x
            shift_y = self.first_detected_hole_pose.position.y - self.hole_pose.position.y
            for pose in pose_array.poses:
                pose.position.x -= shift_x
                pose.position.y -= shift_y
        else:
            self.first_detected_hole_pose = pose_array.poses[0]
        spiral_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        gripper_battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        for pose in spiral_array.poses:
            pose.orientation = gripper_battery_pose.orientation

        # Move above hole with a safe distance
        pose_array = PoseArray()
        pose_array.poses.append(deepcopy(spiral_array.poses[0]))
        pose_array.poses[0].position.z += 0.06
        self.battery_ms.move_group.clear_pose_targets()
        self.battery_ms.move_group.set_pose_reference_frame("base_link")
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        result = self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, vel_scale=0.1, acc_scale=0.1)
        print("Moved to Pose within safe distance")
        if not result:
            rospy.sleep(5)

        rospy.sleep(1)

        # Move vertically down to touch
        pose_array.poses[0].position.z -= 0.1
        self.battery_ms.move_to_touch(pose_array, axis='x', force_thresh=5)
        print("Touched!!!!!")

        rospy.sleep(1)

        print("Start Spiral!!!!")
        # Move spirally until sudden change in z force.
        gripper_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        for pose in spiral_array.poses:
            pose.position.z = gripper_pose.position.z
        self.battery_ms.move_to_touch(spiral_array, axis='x', force_thresh=4)

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
            rospy.sleep(5)

        # Open Gripper
        self.battery_ms.change_tool_status(status=0, signal="Robothon_EE")

        # Return to Capture Pose
        capture_pose = self.tf_services.lookup_transform(
            "base_link", "capture_frame")
        pose_array.poses = [capture_pose]
        self.camera_ms.move_group.clear_pose_targets()
        self.camera_ms.move_group.set_pose_reference_frame("base_link")
        self.camera_ms.move_group.set_pose_target(pose_array.poses[0])
        self.camera_ms.move_group.go()

        rospy.sleep(5)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

    def battery_hole_cb(self, msg):
        pose = msg
        print("Recieved Battery Hole Pose")

        pose.position.x += self.x_comp
        pose.position.y += self.y_comp

        pose_array = PoseArray()
        pose_array.poses.append(pose)
        print("Hole Pose = ", pose)
        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        pose_array.poses[0].position.z += 0.04
        gripper_battery_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_battery")
        pose_array.poses[0].orientation = gripper_battery_pose.orientation
        self.battery_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.battery_ms.move_group.set_pose_target(pose_array.poses[0])
        # self.battery_ms.move_group.set_path_constraints()
        self.battery_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, vel_scale=1, acc_scale=1)
        print("Moved to Pose within safe distance")

        rospy.sleep(1)

        # Open the Gripper
        self.battery_ms.change_tool_status("Robothon_EE", status=0)

        rospy.sleep(1)
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

    def ethernet_cb(self, msg):
        pose_array = msg
        print("Received Ethernet Pose msg")

        # close the gripper
        self.battery_ms.change_tool_status("Robothon_EE", status=1)

        # compensate the error
        for pose in pose_array.poses:
            pose.position.x += self.x_comp
            pose.position.y += self.y_comp

        pose_array.poses[1].position.x += 0.005

        # go to ethernet empty pose
        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        pose_array.poses[0].position.z += 0.03
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
        pose_array.poses[1].orientation = pose_array.poses[0].orientation
        self.ethernet_ms.move_group.set_pose_reference_frame(
            pose_array.header.frame_id)
        self.ethernet_ms.move_group.set_pose_target(pose_array.poses[0])
        self.ethernet_ms.move_group.go()

        rospy.sleep(1)

        # Move to touch to know the actual depth
        ethernet_array = PoseArray()
        ethernet_array.poses.append(deepcopy(pose_array.poses[0]))
        ethernet_array.poses[0].position.z -= 0.05
        self.ethernet_ms.move_to_touch(
            ethernet_array, axis='xy', force_thresh=2)

        rospy.sleep(1)

        # Move up a bit.
        ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        # adjust all poses depth to have the actual measured depth
        for pose in pose_array.poses:
            pose.position.z = ethernet_pose.position.z
        ethernet_pose.position.z += 0.003
        ethernet_array.poses = [ethernet_pose]
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=1, acc_scale=1)
        print("Moved Upwards!!!")
        if not result:
            rospy.sleep(5)

        rospy.sleep(1)

        # open the gripper
        self.battery_ms.change_tool_status("Robothon_EE", status=0)

        rospy.sleep(1)

        # approach the ethernet
        ethernet_array = PoseArray()
        ethernet_array.poses.append(deepcopy(pose_array.poses[1]))
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=0.01, acc_scale=0.01)
        print("Approached Ethernet Cable!!!")
        if not result:
            rospy.sleep(5)

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
            rospy.sleep(5)

        rospy.sleep(1)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

    def ethernet_empty_spiral_cb(self, msg):
        pose_array = msg
        print("Recieved ethernet Pose")
        for pose in pose_array.poses:
            pose.position.x += self.x_comp + 0.005
            pose.position.y += self.y_comp + 0.005

        pose_array = self.tf_services.transform_poses(
            "base_link", "capture_frame", pose_array)
        gripper_ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        for pose in pose_array.poses:
            pose.orientation = gripper_ethernet_pose.orientation

        rospy.sleep(1)

        # move to ethernet empty pose
        ethernet_array = PoseArray()
        ethernet_array.poses.append(deepcopy(pose_array.poses[0]))
        ethernet_array.poses[0].position.z += 0.03
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=0.01, acc_scale=0.01)
        print("Moved To Etherned Empty!!!")
        if not result:
            rospy.sleep(5)

        rospy.sleep(1)

        # Orient the battery move_group to the battery orientation, and move it back a little.
        ethernet_angle = -atan2(1.5, 4.8)
        ethernet_array = self.ethernet_ms.shift_pose_by(self.tf_services, "base_link", "gripper_ethernet", trans=(0, 0, 0),
                                                        rot=(ethernet_angle, 0, 0))
        ethernet_array = self.tf_services.transform_poses(
            "base_link", ethernet_array.header.frame_id, ethernet_array)
        self.ethernet_ms.move_group.set_pose_reference_frame(
            ethernet_array.header.frame_id)
        self.ethernet_ms.move_group.set_pose_target(ethernet_array.poses[0])
        self.ethernet_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, "gripper_battery")
        print("Oriented to ethernet Orientation!!!!")

        rospy.sleep(1)

        # move to touch
        ethernet_array = PoseArray()
        ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        ethernet_array.poses.append(ethernet_pose)
        ethernet_array.poses[0].position.z -= 0.05
        self.ethernet_ms.move_to_touch(
            ethernet_array, axis='xy', force_thresh=5)

        rospy.sleep(1)
        print("Start Spiral!!!!")

        gripper_ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        init_z = gripper_ethernet_pose.position.z
        while fabs(gripper_ethernet_pose.position.z - init_z) <= 0.003:
            for pose in pose_array.poses:
                pose.position.z = gripper_ethernet_pose.position.z
                pose.orientation = ethernet_pose.orientation
            # Move spirally until sudden change in z force.
            self.ethernet_ms.move_to_touch(
                pose_array, axis='xy', force_thresh=9, vel_scale=1, acc_scale=1)
            # move upwards
            ethernet_pose = self.tf_services.lookup_transform(
                "base_link", "gripper_ethernet")
            ethernet_pose.position.z += 0.001
            ethernet_array = PoseArray()
            ethernet_array.poses.append(ethernet_pose)
            result = self.ethernet_ms.move_straight(
                ethernet_array, vel_scale=1, acc_scale=1)
            print("Moved Upwards!!!")
            if not result:
                rospy.sleep(5)
            # move to touch
            ethernet_array = PoseArray()
            ethernet_pose = self.tf_services.lookup_transform(
                "base_link", "gripper_ethernet")
            ethernet_array.poses.append(ethernet_pose)
            ethernet_array.poses[0].position.z -= 0.02
            self.ethernet_ms.move_to_touch(
                ethernet_array, axis='xy', force_thresh=10)
            gripper_ethernet_pose = self.tf_services.lookup_transform(
                "base_link", "gripper_ethernet")

        rospy.sleep(1)

        # Orient the battery move_group to the battery orientation, and move it back a little.
        ethernet_angle = -ethernet_angle
        ethernet_array = self.ethernet_ms.shift_pose_by(self.tf_services, "base_link", "gripper_ethernet", trans=(0, 0, 0),
                                                        rot=(ethernet_angle, 0, 0))
        ethernet_array = self.tf_services.transform_poses(
            "base_link", ethernet_array.header.frame_id, ethernet_array)
        self.ethernet_ms.move_group.set_pose_reference_frame(
            ethernet_array.header.frame_id)
        self.ethernet_ms.move_group.set_pose_target(ethernet_array.poses[0])
        self.ethernet_ms.move_group.go()
        # self.battery_ms.move_straight(pose_array, "gripper_battery")
        print("Oriented to ethernet Orientation!!!!")

        rospy.sleep(1)

        # Move downwards.
        gripper_ethernet_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_ethernet")
        ethernet_array = PoseArray()
        ethernet_array.poses.append(gripper_ethernet_pose)
        ethernet_array.poses[0].position.z -= 0.003
        result = self.ethernet_ms.move_straight(
            ethernet_array, vel_scale=0.01, acc_scale=0.01)
        print("Ethernet inserted!!!")
        if not result:
            rospy.sleep(5)

        rospy.sleep(1)

        # Open Gripper
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
            rospy.sleep(5)
        rospy.sleep(1)

        # Return to Capture Pose
        capture_pose = self.tf_services.lookup_transform(
            "base_link", "capture_frame")
        self.camera_ms.move_group.clear_pose_targets()
        self.camera_ms.move_group.set_pose_reference_frame("base_link")
        self.camera_ms.move_group.set_pose_target(capture_pose)
        self.camera_ms.move_group.go()

        rospy.sleep(5)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)

    def key_cb(self, msg):
        key_pose = msg
        key_pose.position.x += self.x_comp
        key_pose.position.y += self.y_comp

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
        angles[2] = angles[2] + pi
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

        rospy.sleep(1)

        # approach the key
        pose_array = PoseArray()
        pose_array = deepcopy(key_array)
        pose_array.poses[0].position.z -= 0.05
        self.key_ms.move_to_touch(
            pose_array, vel_scale=0.01, acc_scale=0.01, axis='xy', force_thresh=2)
        print("Moved To Key!!!")

        rospy.sleep(1)

        # move upwards
        gripper_key_pose = self.tf_services.lookup_transform(
            "base_link", "gripper_key")
        pose_array = PoseArray()
        pose_array.poses.append(gripper_key_pose)

        pose_array.poses[0].position.z += 0.003
        result = self.key_ms.move_straight(
            pose_array, vel_scale=0.01, acc_scale=0.01)
        print("Moved To Key!!!")
        if not result:
            rospy.sleep(5)

        rospy.sleep(1)

        # close the gripper
        self.key_ms.change_tool_status("Robothon_EE", status=1)

        rospy.sleep(1)

        # move upwards
        pose_array.poses[0].position.z += 0.03
        result = self.key_ms.move_straight(
            pose_array, vel_scale=0.01, acc_scale=0.01)
        print("Moved To Key!!!")
        if not result:
            rospy.sleep(5)

        rospy.sleep(1)

        # move left to drop the key
        pose_array.poses[0].position.y += 0.1
        result = self.key_ms.move_straight(
            pose_array, vel_scale=0.01, acc_scale=0.01)
        print("Moved To Key!!!")
        if not result:
            rospy.sleep(5)

        rospy.sleep(1)

        # open the gripper
        self.key_ms.change_tool_status("Robothon_EE", status=0)

        rospy.sleep(1)

        # publish done msgmsgne"
        pb_done_msg = String()
        pb_done_msg.data = "done"
        self.done_pub.publish(pb_done_msg)


if __name__ == "__main__":
    rospy.init_node("competition_operations")
    rospy.sleep(1)
    comp_op = CompetitionOperations()
    rospy.spin()
