#!/usr/bin/env python

from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import MoveGroupActionFeedback
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import String
import numpy as np
from cut_contour.robot_helpers import TransformServices
import rospy
from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_matrix
import tf

class Motion():

    TOOL_GROUP = "camera_group"

    def __init__(self):
        self.camera_group = MoveGroupCommander(self.TOOL_GROUP)
        self.tf_services = TransformServices()
        self.transformer_listener = tf.TransformListener()
        self.state_publisher = rospy.Publisher(
            "/capture_state", String, queue_size=1)
        
        rospy.Subscriber("/desired_pose_topic", PoseStamped, self.desired_pose_cb)
        
        #send forst message
        rospy.sleep(2)
        self.capture_state_msg = String()
        self.capture_state_msg.data = "Orient"
        self.state_publisher.publish(self.capture_state_msg)
        self.catpure_pose = None
        
    
    def desired_pose_cb(self, camera_detect_pose):
        #Get Pose with respect to base_link
        # print("desired_poses")
        # calibrated_to_world = self.tf_services.lookup_transform('base_link','calibrated_frame')
        # calibrated_to_camera = self.tf_services.lookup_transform('camera_link','calibrated_frame')
        
        # pose_rotation_matrix = quaternion_matrix([camera_detect_pose.pose.orientation.w,
        #                                                 camera_detect_pose.pose.orientation.x,
        #                                                 camera_detect_pose.pose.orientation.y,
        #                                                 camera_detect_pose.pose.orientation.z])
        # calibrated_to_world_rotation_matrix = quaternion_matrix([calibrated_to_world.orientation.w,
        #                                                 calibrated_to_world.orientation.x,
        #                                                 calibrated_to_world.orientation.y,
        #                                                 calibrated_to_world.orientation.z])
        # calibrated_to_camera_rotation_matrix = quaternion_matrix([calibrated_to_camera.orientation.w,
        #                                                 calibrated_to_camera.orientation.x,
        #                                                 calibrated_to_camera.orientation.y,
        #                                                 calibrated_to_camera.orientation.z])
        
        # pose_tf_matrix = np.zeros((4,4))
        # calibrated_to_world_tf_matrix = np.zeros((4,4))
        # calibrated_to_camera_tf_matrix = np.zeros((4,4))
        
        # pose_tf_matrix = pose_rotation_matrix
        # print(pose_tf_matrix)
        # pose_tf_matrix[:,3] = np.array([camera_detect_pose.pose.position.x,
        #                                 camera_detect_pose.pose.position.y,
        #                                 camera_detect_pose.pose.position.z,
        #                                 1])
        
        # calibrated_to_world_tf_matrix = calibrated_to_world_rotation_matrix
        # calibrated_to_world_tf_matrix[:,3] = np.array([calibrated_to_world.position.x,
        #                                                calibrated_to_world.position.y,
        #                                                calibrated_to_world.position.z,
        #                                                1])
        
        # calibrated_to_camera_tf_matrix = calibrated_to_camera_rotation_matrix
        # calibrated_to_camera_tf_matrix[:,3] = np.array([calibrated_to_camera.position.x,
        #                                                 calibrated_to_camera.position.y,
        #                                                 calibrated_to_camera.position.z,
        #                                                 1])
        
        # total_transformation_matrix = calibrated_to_camera_tf_matrix.dot(calibrated_to_world_tf_matrix.dot(pose_tf_matrix))
        
        # # # desired Pose
        # final_pose = Pose()
        # w , x , y, z = quaternion_from_matrix(total_transformation_matrix)
        # final_pose.position.x = total_transformation_matrix[0,3]
        # final_pose.position.y = total_transformation_matrix[1,3]
        # final_pose.position.z = total_transformation_matrix[2,3]
        # final_pose.orientation.w = w
        # final_pose.orientation.x = x
        # final_pose.orientation.y = y
        # final_pose.orientation.z = z
        
        # go to pose
        self.transformer_listener.waitForTransform(
                'base_link', 'calibrated_frame', rospy.Time(), rospy.Duration(1))
        trans_pose = self.transformer_listener.transformPose(
                'base_link', camera_detect_pose)
        print(trans_pose.pose)
        self.camera_group.set_pose_reference_frame('base_link')
        self.camera_group.clear_pose_targets()
        self.camera_group.set_pose_target(trans_pose.pose)
        approach_result = self.camera_group.go()
        rospy.sleep(1)
        
        # save capture pose
        self.catpure_pose = self.tf_services.create_frame(
            ref_frame="base_link", moving_frame="calibrated_frame", new_frame="capture_frame")
        
        self.capture_state_msg.data = "Detect"
        self.state_publisher.publish(self.capture_state_msg)
        return

if __name__ == "__main__":

    rospy.init_node("competition_control")
    controller = Motion()
    rospy.sleep(1)
    rospy.spin()
