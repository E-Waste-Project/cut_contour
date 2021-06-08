#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Pose
from geometry_msgs.msg import TwistStamped
import numpy as np
import tf
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest 
from std_msgs.msg import Float64
from std_msgs.msg import String
import matplotlib.pyplot as plt
from cut_contour import irb120
#from mpl_toolkits.mplot3d import axes3d
from tf.transformations import euler_from_quaternion
import roslaunch
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest
from abb_robot_msgs.msg import SystemState

class Subscribers():
    def __init__(self):
        self.current_index = 0
        self.pose_array = None
        self.pose_flag = False
        self.working_flag = False
        self.feed_rate = 0.002
        self.completed_flag = False
        self.kp = 12.0
        self.min_dist = 0
        self.control_action = 0
        self.position_control_action = 0
        rospy.Subscriber(
            "/cut_xyz", PoseArray, self.poses_callback)
        
        self.transformer_listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(1.0/100.0) ,self.timer_callback)
        rospy.Timer(rospy.Duration(1.0/2.0) ,self.output_callback)
        self.output_pub = rospy.Publisher("/servo_server/delta_twist_cmds",TwistStamped,latch=True,queue_size=1)
        self.dist = 0
        #self.controller_client = rospy.ServiceProxy("/egm/controller_manager/switch_controller", SwitchController)
        # self.cutting_tool_group = MoveGroupCommander("cutting_tool")
        self.robot = irb120()
        self.dist_threshold = 0.003
        self.theta = 0
        self.setpoint_pub = rospy.Publisher("/position/setpoint",Float64,latch=True,queue_size=1)
        
        self.error_pub = rospy.Publisher("/position/state",Float64,latch=True,queue_size=1)
        self.action_sub = rospy.Subscriber("/force_pid/control_effort",Float64,self.force_control_action_callback)
        self.currentPos_pub = rospy.Publisher("/current_pos",PoseStamped,queue_size=1)
        self.desiredPos_pub = rospy.Publisher("/desired_pos",PoseStamped,queue_size=1)
        self.position_action_sub = rospy.Subscriber("/position/control_effort",Float64,self.position_control_action_callback)
        self.pitch_pub = rospy.Publisher("/pitch/state", Float64, queue_size=1)
        self.done_pub = rospy.Publisher("/done", String, queue_size=1)
        rospy.Subscriber("pitch/control_effort", Float64, self.pitch_control_cb)
        rospy.Subscriber("/rws/system_states", SystemState, self.states_cb)
        self.x_list =[]
        self.y_list =[]
        self.z_list =[]
        #self.fig = plt.figure()
        self.curr_x = []
        self.curr_y = []
        self.pitch = 0
        self.angular_control_action = 0
        self.z_offset = 0
          
        self.depth_of_cut = 0.035     
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.servo_launch = roslaunch.parent.ROSLaunchParent(
    uuid, ["/home/zaferpc/abb_ws/src/cut_contour/launch/servo_test.launch"])
        self.servo_launch.start()
    
        self.cutting_tool = rospy.ServiceProxy("/rws/set_io_signal", SetIOSignal)  
    
    def states_cb(self, state_msg):
        if state_msg.motors_on == False and self.working_flag == True:
            failed_msg = String()
            failed_msg.data = "cutting failed"
            self.done_pub.publish(failed_msg)    
          
    def pitch_control_cb(self, msg):
        self.angular_control_action = msg.data
        
    def position_control_action_callback(self, msg):
        self.position_control_action = msg.data
        
    def force_control_action_callback(self, msg):
        self.control_action = msg.data
        
    def change_controller(self, start, stop):
        change_req = SwitchControllerRequest()
        change_req.start_controllers.append(start)
        change_req.stop_controllers.append(stop)
        change_req.strictness = change_req.STRICT
        change_req.start_asap = change_req.BEST_EFFORT
        self.controller_client.wait_for_service()
        response = self.controller_client(change_req)
        return response
        
    def transform_contour(self, target_frame, source_frame, contour):
        print ("here")
        trans_contour = PoseArray()
        self.x_list =[]
        self.y_list =[]
        self.z_list =[]
        for i in range(len(contour.poses)):
            point = PointStamped()
            trans_pose = Pose()
            point.point.x = contour.poses[i].position.x
            point.point.y = contour.poses[i].position.y
            point.point.z = contour.poses[i].position.z
            point.header.frame_id = source_frame
            self.transformer_listener.waitForTransform(target_frame, source_frame, rospy.Time(),rospy.Duration(1))
            trans_point = self.transformer_listener.transformPoint(target_frame, point)
            trans_pose.position.x = trans_point.point.x
            self.x_list.append(trans_pose.position.x)
            trans_pose.position.y = trans_point.point.y
            self.y_list.append(trans_pose.position.y)
            trans_pose.position.z = trans_point.point.z
            self.z_list.append(trans_pose.position.z)
            trans_contour.poses.append(trans_pose)
            
            desiredPos_msg = PoseStamped()
            desiredPos_msg.pose.position.x = trans_pose.position.x
            desiredPos_msg.pose.position.y = trans_pose.position.y
            desiredPos_msg.header.stamp = rospy.Time.now()
            self.desiredPos_pub.publish(desiredPos_msg)
            
        # fig = plt.figure()
        # ax = plt.axes(projection="3d")
        # ax = fig.gca(projection='3d')
        # ax.plot(self.x_list, self.y_list, self.z_list)
        #plt.plot(self.x_list,self.y_list)
        #plt.ion()
        #plt.show()
        #plt.show(block=False)
        trans_contour.header.frame_id = target_frame
        trans_contour.header.stamp = rospy.Time()
        return trans_contour
        
    def poses_callback(self, msg):
        
        self.working_flag = True
       # self.servo_launch.start()
        trans_contour = PoseArray()
        trans_contour = self.transform_contour("base_link", "camera_color_optical_frame", msg)
        # trans_contour = msg

        self.pose_array = np.zeros((len(trans_contour.poses),3))
        for pose_index in range(0,len(trans_contour.poses)):
            self.pose_array[pose_index,0] = trans_contour.poses[pose_index].position.x
            self.pose_array[pose_index,1] = trans_contour.poses[pose_index].position.y
            self.pose_array[pose_index,2] = trans_contour.poses[pose_index].position.z

        # temp_array = np.unique(self.pose_array, axis=0)
        # self.pose_array = None
        # self.pose_array = temp_array
        # print(self.pose_array.shape)
                
        self.robot.group.set_pose_reference_frame("base_link")
        x = self.pose_array[0,0]
        y = self.pose_array[0,1]
        z = self.pose_array[0,2]
        
        print("x = ", x)
        print("y = ", y)
        print("z = ", z)
        print(self.pose_array.shape)

        self.robot.group.set_pose_target([x, y, z + self.z_offset, 0, 1, 0, 0])
        self.robot.group.go()
        
        # cutting tool on
        cutting_tool_on = SetIOSignalRequest()
        cutting_tool_on.signal = "CuttingTool"
        cutting_tool_on.value = "1"
        self.cutting_tool(cutting_tool_on)
        
        goal_list = PoseArray()
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z - self.depth_of_cut
        #goal.position.z = z+z_offset
        goal.orientation.x = 0
        goal.orientation.y = 1
        goal.orientation.z = 0
        goal.orientation.w = 0
        goal_list.poses.append(goal)
        self.robot.cut_in_contour(goal_list.poses, "base_link", vel_scale=0.001, acc_scale=0.001)
        # temp_array = np.delete(self.pose_array,0, 0)
        # self.pose_array = None
        # self.pose_array = temp_array      
        #res = self.change_controller("joint_group_velocity_controller", "joint_trajectory_controller")
       # print("switching controller succeeded = ", res)
        setpoint_msg = Float64()
        #setpoint_msg.data = 0
        setpoint_msg.data = goal.position.z
        self.setpoint_pub.publish(setpoint_msg)
        self.pose_flag = True
        print("CUT XYZ Received")
           
    def timer_callback(self,event):
        if(self.pose_flag == True):
            source_frame = "tool0"
            target_frame = "base_link"
            self.transformer_listener.waitForTransform(target_frame, source_frame, rospy.Time(),rospy.Duration(0.001))
            trans, rot = self.transformer_listener.lookupTransform(target_frame, source_frame ,rospy.Time())
            position_msg = PoseStamped()
            position_msg.pose.position.x = trans[0]
            position_msg.pose.position.y = trans[1]
            position_msg.header.stamp = rospy.Time.now()
            self.currentPos_pub.publish(position_msg)
            self.curr_x.append(trans[0])
            self.curr_y.append(trans[1])
            _, self.pitch, _ = euler_from_quaternion(rot)
            pitch_msg = Float64(data=self.pitch)
            self.pitch_pub.publish(pitch_msg)
            # plt.scatter(trans[0],trans[1],c='r')

            # # plt.figure(1)
            # plt.draw()
            # plt.pause(0.001)
            
            # min_index = np.argmin((self.pose_array[:,0] - trans[0])**2 + (self.pose_array[:,1] - trans[1])**2)
            # self.min_dist = np.min((self.pose_array[:,0] - trans[0])**2 + (self.pose_array[:,1] - trans[1])**2)
            
            # self.theta = np.pi + np.arctan2((trans[1] - self.pose_array[min_index, 1]), (trans[0] - self.pose_array[min_index, 0]))
            self.theta =  np.arctan2((self.pose_array[self.current_index, 1] - trans[1] ), (self.pose_array[self.current_index, 0] - trans[0]))
            z = self.pose_array[self.current_index, 2] - self.depth_of_cut
            setpoint_msg = Float64()
            setpoint_msg.data = z
            self.setpoint_pub.publish(setpoint_msg)
            # temp_array = np.delete(self.pose_array,min_index, 0)
            # self.pose_array = None
            # self.pose_array = temp_array
            # print(self.pose_array.shape)
            # print("current position = ", trans)
            #self.dist = np.math.sqrt((trans[0]-self.pose_array[self.min_index,0])**2 + (trans[1]-self.pose_array[self.min_index,1])**2)
            self.dist = np.math.sqrt((trans[0]-self.pose_array[self.current_index,0])**2 + (trans[1]-self.pose_array[self.current_index,1])**2)
            error_msg = Float64()
            #error_msg.data = self.dist
            error_msg.data = trans[2]
            self.error_pub.publish(error_msg)
            if self.dist <= self.dist_threshold:
                self.current_index = self.current_index + 1
                if self.current_index >= self.pose_array.shape[0]:
                    self.current_index = 0
                    self.pose_array = None
                    self.pose_flag = False
                    self.completed_flag = True
                #    res = self.change_controller("joint_trajectory_controller", "joint_group_velocity_controller")
                #    print("switching controller succeeded = ", res)
                    
                    
    def output_callback(self,event):
        
        message_output = TwistStamped()
        if(self.pose_flag == True):
            
            
            message_output.header.stamp = rospy.Time.now()
            message_output.header.frame_id = 'base_link' 
            
            # message_output.twist.linear.x = self.kp * np.math.sqrt(self.min_dist) * np.cos(self.theta)
            # message_output.twist.linear.y = self.kp * np.math.sqrt(self.min_dist) * np.sin(self.theta)
            
            # message_output.twist.linear.x = self.kp * self.dist * np.cos(self.theta)
            # message_output.twist.linear.y = self.kp * self.dist * np.sin(self.theta)
            
            #message_output.twist.linear.x = self.control_action * np.cos(self.theta)
            #message_output.twist.linear.y = self.control_action * np.sin(self.theta)
            # vel_limit = 0.01
            # if self.control_action != 0:
            #     self.control_action = max(self.control_action, vel_limit) if self.control_action > 0 else min(self.control_action, -vel_limit)
            # message_output.twist.linear.x = self.control_action * np.cos(self.theta)
            # message_output.twist.linear.y = self.control_action * np.sin(self.theta)
            
            message_output.twist.linear.x = ((self.feed_rate + self.control_action)) * np.cos(self.theta)
            message_output.twist.linear.y = ((self.feed_rate + self.control_action)) * np.sin(self.theta)
            
            # message_output.twist.linear.x = (self.feed_rate) * np.cos(self.theta)
            # message_output.twist.linear.y = (self.feed_rate) * np.sin(self.theta)
                       
            message_output.twist.linear.z = self.position_control_action
            message_output.twist.angular.x = 0
            message_output.twist.angular.y = self.angular_control_action
            message_output.twist.angular.z = 0
            
            print("NON-zero velocities")
            print("current_dist = ", self.dist)
            print("current index = ", self.current_index)
            print ("velocity x = ", message_output.twist.linear.x)
            print ("velocity y = ", message_output.twist.linear.y)
            self.output_pub.publish(message_output)
            
        else:
            if self.completed_flag == False:
                message_output.twist.linear.x = 0
                message_output.twist.linear.y = 0
            
                message_output.twist.linear.z = 0
                message_output.twist.angular.x = 0
                message_output.twist.angular.y = 0
                message_output.twist.angular.z = 0
                self.output_pub.publish(message_output)
            if self.completed_flag == True:
                source_frame = "tool0"
                target_frame = "base_link"
                print("Returning to start position")
                self.servo_launch.shutdown()
                rospy.sleep(5)
                self.transformer_listener.waitForTransform(target_frame, source_frame, rospy.Time(),rospy.Duration(0.001))
                trans, rot = self.transformer_listener.lookupTransform(target_frame, source_frame ,rospy.Time())
                self.robot.group.set_pose_target([trans[0],trans[1], trans[2]+0.05, 0, 1, 0, 0])
                self.robot.group.go()
                
                # cutting tool off
                self.cutting_tool.wait_for_service()
                cutting_tool_on = SetIOSignalRequest()
                cutting_tool_on.signal = "CuttingTool"
                cutting_tool_on.value = "0"
                self.cutting_tool(cutting_tool_on)
                
                source_frame = "camera_start"
                target_frame = "base_link"
                rospy.sleep(1)
                self.transformer_listener.waitForTransform(target_frame, source_frame, rospy.Time(),rospy.Duration(0.001))
                trans, rot = self.transformer_listener.lookupTransform(target_frame, source_frame ,rospy.Time())

                self.robot.group.set_pose_target([trans[0],trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])
                self.robot.group.go()
                
                self.working_flag = False
            #rospy.sleep(10)
                done_msg = String()
                done_msg.data = "Done"
                self.done_pub.publish(done_msg)
                self.completed_flag =False

 

if __name__ == "__main__":
    rospy.init_node("motion_calculator")
    subs = Subscribers()
    rospy.spin()
    plt.plot(subs.x_list,subs.y_list)
    plt.plot(subs.curr_x,subs.curr_y,c='r')
    plt.legend(['Correct Path','Executed Path'])
    plt.show()