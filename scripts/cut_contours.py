#!/usr/bin/env python
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, WrenchStamped
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest
from abb_robot_msgs.msg import SystemState
from std_msgs.msg import Float64
from std_msgs.msg import String
from moveit_msgs.msg import MoveGroupActionFeedback
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import rospy
from copy import deepcopy
import tf
from math import fabs
from cut_contour.robot_helpers import transform_poses


class irb120():

    TOOL_GROUP = "cutting_tool"

    def __init__(self):
        self.group = MoveGroupCommander(self.TOOL_GROUP)
        self.transformer_listener = tf.TransformListener()
        rospy.Subscriber("/cut_xyz", PoseArray, self.cut_cb)
        rospy.Subscriber("/rws/system_states", SystemState, self.states_cb)
        self.done_pub = rospy.Publisher("/done", String, queue_size=1)
        self.approach_dist = 0.01
        self.cut_feed_rate = 0.005
        self.retreat_feed_rate = 0.5
        self.measure_z_feed_rate = 0.0005
        self.depth_of_cut = 0.005
        self.retreat_dist = 0.03
        self.z_to_measure = -0.03
        self.force_z_threshold = 1.0
        self.working_flag = False
        self.tool_quat_base_link = [0, 1, 0, 0]
        self.tool_quat_table_link = [0.707, -0.707, 0.000, -0.000]
        self.cutting_tool = rospy.ServiceProxy(
            "/rws/set_io_signal", SetIOSignal)

    def states_cb(self, state_msg):
        if state_msg.motors_on == False and self.working_flag == True:
            failed_msg = String()
            failed_msg.data = "cutting failed"
            self.done_pub.publish(failed_msg)

    def approch_point(self, point_location, ref_frame="base_link"):

        self.group.set_pose_reference_frame(ref_frame)
        # approach
        set_point = Pose()
        set_point.position.x = point_location.position.x
        set_point.position.y = point_location.position.y
        set_point.position.z = point_location.position.z + self.approach_dist

        quat = self.tool_quat_base_link

        if ref_frame == "table_link":
            quat = self.tool_quat_table_link

        set_point.orientation.x = quat[0]
        set_point.orientation.y = quat[1]
        set_point.orientation.z = quat[2]
        set_point.orientation.w = quat[3]

        self.group.clear_pose_targets()
        self.group.set_pose_target(set_point)
        approach_result = self.group.go()
        return approach_result

    def move_straight(self, point_location, z_dist=0, ref_frame="base_link", vel_scale=1, acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0):
        # set the pose_arr reference frame
        self.group.set_pose_reference_frame(ref_frame)

        # compute the plan
        goal_pose_arr = []
        set_point = Pose()
        set_point.position.x = point_location.position.x
        set_point.position.y = point_location.position.y
        set_point.position.z = point_location.position.z + z_dist
        quat = self.tool_quat_base_link
        if ref_frame == "table_link":
            quat = self.tool_quat_table_link
        set_point.orientation.x = quat[0]
        set_point.orientation.y = quat[1]
        set_point.orientation.z = quat[2]
        set_point.orientation.w = quat[3]

        goal_pose_arr.append(set_point)
        plan, fraction = self.group.compute_cartesian_path(
            goal_pose_arr, eef_step, jump_threshold, avoid_collisions)
        print(fraction)

        # filter the output plan
        filtered_plan = self.__filter_plan(plan)
        # filtered_plan = plan

        # Time paremetrize the path to generate trajectory.
        final_traj = self.group.retime_trajectory(
            self.group.get_current_state(), filtered_plan, vel_scale, acc_scale)
        
        # execute the filtered plan
        result = self.group.execute(final_traj)
        
        return result

    def safe_move_straight(self, point_location, z_dist=0, ref_frame="base_link", vel_scale=1, acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0):
        # set the pose_arr reference frame
        self.group.set_pose_reference_frame(ref_frame)
        goal_state_msg = rospy.wait_for_message(
            "/move_group/status", GoalStatusArray)
        goal_state = goal_state_msg.status_list[-1].status
        prev_goal_id = goal_state_msg.status_list[-1].goal_id.id
        # compute the plan
        goal_pose_arr = []
        set_point = Pose()
        set_point.position.x = point_location.position.x
        set_point.position.y = point_location.position.y
        set_point.position.z = point_location.position.z + z_dist

        quat = self.tool_quat_base_link
        if ref_frame == "table_link":
            quat = self.tool_quat_table_link
        set_point.orientation.x = quat[0]
        set_point.orientation.y = quat[1]
        set_point.orientation.z = quat[2]
        set_point.orientation.w = quat[3]

        goal_pose_arr.append(set_point)
        plan, fraction = self.group.compute_cartesian_path(
            goal_pose_arr, eef_step, jump_threshold, avoid_collisions)
        print(fraction)

        # filter the output plan            prev_goal_id = curr_goal_id

        filtered_plan = self.__filter_plan(plan)

        # get the initial force
        msg = rospy.wait_for_message(
            "/ft_sensor_wrench/resultant/filtered", Float64)
        init_force = msg.data
        print("initial_force = ", init_force)

        # execute the filtered plan
        final_traj = self.group.retime_trajectory(
            self.group.get_current_state(), filtered_plan, vel_scale, acc_scale)
        self.group.execute(final_traj, wait=False)
        # goal_state_msg = rospy.wait_for_message("/move_group/status", GoalStatusArray)
        # print(len(goal_state_msg.status_list))
        # goal_state = goal_state_msg.status_list[-1].status
        # print(goal_state)
        # while goal_state != GoalStatus.ACTIVE:
        #     continue
        current_force = init_force
        change_force = fabs(current_force - init_force)
        goal_state = GoalStatus()
        while change_force < (self.force_z_threshold - 0.5):
            goal_state_msg = rospy.wait_for_message(
                "/move_group/status", GoalStatusArray)
            goal_state = goal_state_msg.status_list[-1].status
            curr_goal_id = goal_state_msg.status_list[-1].goal_id.id
            print(len(goal_state_msg.status_list))
            print(goal_state)
            if goal_state == GoalStatus.SUCCEEDED and (curr_goal_id != prev_goal_id):
                break
            msg = rospy.wait_for_message(
                "/ft_sensor_wrench/resultant/filtered", Float64)
            current_force = msg.data
            change_force = fabs(current_force - init_force)
            print("change in force = ", change_force)

        if goal_state != GoalStatus.SUCCEEDED:
            self.group.stop()
        # get the new path point
        pose_arr = PoseArray()
        tool_pose = PoseArray()
        pose = Pose()
        print(pose)
        pose_arr.header.frame_id = "tool0_comp"
        pose_arr.poses.append(pose)
        tool_pose = transform_poses("base_link", "tool0_comp", pose_arr)

        return tool_pose.poses[0]

    def cut_contour_seq(self, contour):
        # approach the first point
        first_point = self.calculate_center(contour)
        result = self.approch_point(first_point)
        print("approach result = ", result)

        # measure z
        rospy.sleep(3)
        measure_point = deepcopy(first_point)
        surface_z = self.measure_z(
            measure_point, vel_scale=self.measure_z_feed_rate, acc_scale=self.measure_z_feed_rate)

        # modify z of the contour
        for i in range(len(contour.poses)):
            contour.poses[i].position.z = surface_z

        # retreat
        retreat_from_measure = deepcopy(contour.poses[0])
        result = self.move_straight(
            retreat_from_measure, z_dist=0.01, vel_scale=1, acc_scale=1)
        print("retreat result = ", result)

        # turn on the tool
        # self.change_tool_status(status=1)

        # move in the air above contour to check for obstacles and modify path to avoid them
        for i in range(len(contour.poses)):
            point_to_cut = deepcopy(contour.poses[i])
            result = self.safe_move_straight(
                point_to_cut, z_dist=self.depth_of_cut, vel_scale=self.cut_feed_rate, acc_scale=self.cut_feed_rate)
        print("contour result = ", result)

        # turn on the tool
        # self.change_tool_status(status=1)

        # take the depth of cut and cut the contour
        # for i in range(len(contour.poses)):
        #     point_to_cut = deepcopy(contour.poses[i])
        #     result = self.move_straight(
        #         point_to_cut, z_dist=self.depth_of_cut, vel_scale=self.cut_feed_rate, acc_scale=self.cut_feed_rate)
        # print ("contour result = ", result)

        # # retreat
        # retreat_point = deepcopy(contour.poses[-1])
        # result = self.move_straight(
        #     point_to_cut, z_dist=self.retreat_dist, vel_scale=self.retreat_feed_rate, acc_scale=self.retreat_feed_rate)
        # print ("retreat result = ", result)

        # turn off the tool
        self.change_tool_status(status=0)
        rospy.sleep(6)

    def __filter_plan(self, plan):
        last_time_step = plan.joint_trajectory.points[0].time_from_start.to_sec
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header = plan.joint_trajectory.header
        new_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
        new_plan.joint_trajectory.points.append(
            plan.joint_trajectory.points[0])
        for i in range(1, len(plan.joint_trajectory.points)):
            point = plan.joint_trajectory.points[i]
            if point.time_from_start.to_sec > last_time_step:
                new_plan.joint_trajectory.points.append(point)
            last_time_step = point.time_from_start.to_sec
        return new_plan

    def measure_z(self, point_location, ref_frame="base_link", vel_scale=1, acc_scale=1, avoid_collisions=True, eef_step=0.0001, jump_threshold=0.0):
        # set the pose_arr reference frame
        self.group.set_pose_reference_frame(ref_frame)

        # compute the plan
        goal_pose_arr = []
        set_point = Pose()
        set_point.position.x = point_location.position.x
        set_point.position.y = point_location.position.y
        set_point.position.z = point_location.position.z + self.z_to_measure

        quat = self.tool_quat_base_link
        if ref_frame == "table_link":
            quat = self.tool_quat_table_link
        set_point.orientation.x = quat[0]
        set_point.orientation.y = quat[1]
        set_point.orientation.z = quat[2]
        set_point.orientation.w = quat[3]

        goal_pose_arr.append(set_point)
        plan, fraction = self.group.compute_cartesian_path(
            goal_pose_arr, eef_step, jump_threshold, avoid_collisions)
        print(fraction)

        # filter the output plan    def cut_cb(self, contour_msg):
        filtered_plan = self.__filter_plan(plan)

        # get the initial force
        msg = rospy.wait_for_message("/ft_sensor_wrench/filtered_z", Float64)
        init_force_z = msg.data
        print("initial_force = ", init_force_z)

        # execute the filtered plan
        final_traj = self.group.retime_trajectory(
            self.group.get_current_state(), filtered_plan, vel_scale, acc_scale)
        self.group.execute(final_traj, wait=False)

        current_force_z = init_force_z
        change_force = fabs(current_force_z - init_force_z)
        while change_force < self.force_z_threshold:
            msg = rospy.wait_for_message(
                "/ft_sensor_wrench/filtered_z", Float64)
            current_force_z = msg.data
            change_force = fabs(current_force_z - init_force_z)
            print("change in force = ", change_force)

        self.group.stop()
        # get the current z
        pose_arr = PoseArray()
        tool_pose = PoseArray()
        pose = Pose()
        print(pose)
        pose_arr.header.frame_id = "tool0_comp"
        pose_arr.poses.append(pose)
        tool_pose = transform_poses("base_link", "tool0_comp", pose_arr)

        return tool_pose.poses[0].position.z

    def calculate_center(self, contour):
        center = Pose()
        center = deepcopy(contour.poses[0])
        npoints = len(contour.poses)
        for i in range(1, npoints):
            center.position.x += contour.poses[i].position.x
            center.position.y += contour.poses[i].position.y
        center.position.x /= npoints
        center.position.y /= npoints
        return center

    def change_tool_status(self, status=0):
        self.cutting_tool.wait_for_service()
        cutting_tool_status = SetIOSignalRequest()
        cutting_tool_status.signal = "CuttingTool"
        cutting_tool_status.value = str(status)
        response = self.cutting_tool(cutting_tool_status)
        return response

    def cut_cb(self, contour_msg):
        self.working_flag = True
        trans_poses = transform_poses(
            "base_link", "calibrated_frame", contour_msg)
        no_contours = len(contour_msg.poses) / 5
        # no_contours = 1
        for i in range(no_contours):
            contour_to_cut = PoseArray()
            contour_to_cut.header = trans_poses.header
            contour_to_cut.poses = deepcopy(trans_poses.poses[i*5:i*5+5])
            # contour_to_cut.poses = deepcopy(trans_poses.poses[:])
            # print contour_to_cut
            # raw_input()
            self.cut_contour_seq(contour_to_cut)
        self.working_flag = False
        done_msg = String()
        done_msg.data = "Done"
        self.done_pub.publish(done_msg)


if __name__ == "__main__":

    rospy.init_node("cut_contour")
    robot = irb120()
    rospy.sleep(1)
    rospy.spin()
