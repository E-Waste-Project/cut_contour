#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import message_filters


actual_1_list = []
actual_2_list = []
actual_3_list = []
actual_4_list = []
actual_5_list = []
actual_6_list = []
ac_time_list = []

setpoint_1_list = []
setpoint_2_list = []
setpoint_3_list = []
setpoint_4_list = []
setpoint_5_list = []
setpoint_6_list = []
sp_time_list = []

def actual_cb(actual_msg):
    actual_1_list.append(actual_msg.velocity[0])
    actual_2_list.append(actual_msg.velocity[1])
    actual_3_list.append(actual_msg.velocity[2])
    actual_4_list.append(actual_msg.velocity[3])
    actual_5_list.append(actual_msg.velocity[4])
    actual_6_list.append(actual_msg.velocity[5])
    ac_time_list.append(rospy.Time.now().to_sec())

def setpoint_cb(setpoint_msg):
    setpoint_1_list.append(setpoint_msg.points[0].velocities[0])
    setpoint_2_list.append(setpoint_msg.points[0].velocities[1])
    setpoint_3_list.append(setpoint_msg.points[0].velocities[2])
    setpoint_4_list.append(setpoint_msg.points[0].velocities[3])
    setpoint_5_list.append(setpoint_msg.points[0].velocities[4])
    setpoint_6_list.append(setpoint_msg.points[0].velocities[5])
    sp_time_list.append(rospy.Time.now().to_sec())


rospy.init_node("joint_vel_plot")
rospy.Subscriber("/joint_states", JointState, actual_cb)
rospy.Subscriber("/egm/joint_trajectory_controller/command", JointTrajectory, setpoint_cb)
rospy.spin()


first_time_sp = sp_time_list[0]
for i in range(len(sp_time_list)):
    sp_time_list[i] = sp_time_list[i] - first_time_sp


first_time_ac = ac_time_list[0]
for i in range(len(ac_time_list)):
    ac_time_list[i] = ac_time_list[i] - first_time_ac

# plotting
fig = plt.figure(1)
plt.plot(ac_time_list, actual_1_list, c='r')
plt.plot(sp_time_list, setpoint_1_list, c='b')

fig = plt.figure(2)
plt.plot(ac_time_list, actual_2_list, c='r')
plt.plot(sp_time_list, setpoint_2_list, c='b')

fig = plt.figure(3)
plt.plot(ac_time_list, actual_3_list, c='r')
plt.plot(sp_time_list, setpoint_3_list, c='b')

fig = plt.figure(4)
plt.plot(ac_time_list, actual_4_list, c='r')
plt.plot(sp_time_list, setpoint_4_list, c='b')

fig = plt.figure(5)
plt.plot(ac_time_list, actual_5_list, c='r')
plt.plot(sp_time_list, setpoint_5_list, c='b')

fig = plt.figure(6)
plt.plot(ac_time_list, actual_6_list, c='r')
plt.plot(sp_time_list, setpoint_6_list, c='b')

plt.show()
