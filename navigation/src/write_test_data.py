#! /usr/bin/env python3
from math import pi
import rospy
from std_msgs.msg import Int64, Float32
import pandas as pd
import matplotlib.pyplot as plt


class Diff_drive_kinematics():
    def __init__(self):
        self.vl_sub = rospy.Subscriber('/topicV_L', Int64, self.callback_L)
        self.vr_sub = rospy.Subscriber('/topicV_R', Int64, self.callback_R)
        self.right_wheel_pub = rospy.Publisher(
            '/control_right_wheel/command', Float32, queue_size=1)
        self.left_wheel_pub = rospy.Publisher(
            '/control_left_wheel/command', Float32, queue_size=1)
        self.vl = 0
        self.vr = 0
        self.rpmL = 8
        self.rpmR = 8
        self.rate = rospy.Rate(1)
        self.left_wheel_pub.publish(self.rpmL)
        self.right_wheel_pub.publish(self.rpmR)

    def callback_L(self, msg):
        self.vl = msg.data

    def callback_R(self, msg):
        self.vr = msg.data

    def convert_vels(self):
        
# set
plt.plot(,self.rpmL)

# vl
plt.plot(,self.vl)
 
# vr
plt.plot(, self.vr, color='green')
 
# title
plt.title('PID test')

# towards right
plt.xticks(rotation=30, ha='right')
 
# Giving x and y label to the graph
plt.xlabel('Date')
plt.ylabel('Classes')


if __name__ == '__main__':
    rospy.init_node('convert_vel_node', anonymous=True)
    diffdrive_object = Diff_drive_kinematics()
    diffdrive_object.pub_wheel_vels()