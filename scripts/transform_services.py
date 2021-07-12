#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cut_contour.robot_helpers import TransformServices


class TFServicesROSInterface(TransformServices):
    def __init__(self):
        super().__init__()
        
        rospy.Subscriber("/create_frame_at_pose", PoseStamped,
                         callback=self.create_frame_at_pose)
        self.create_frame_at_pose_done = rospy.Publisher(
            "/create_frame_at_pose_done", String, queue_size=1)

        
    def create_frame_at_pose(self, pose_msg):
        pose = pose_msg.pose
        new_frame = pose_msg.header.frame_id
        _ = super().create_frame_at_pose(pose, "base_link", new_frame)
        done_msg = String()
        done_msg.data = "done"
        self.create_frame_at_pose_done.publish(done_msg)


if __name__ == "__main__":
    rospy.init_node("tf_services_ros_interface")
    tf_services_interface = TFServicesROSInterface()
    rospy.spin()
