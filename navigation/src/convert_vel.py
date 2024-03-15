#! /usr/bin/env python3
from math import pi
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class Diff_drive_kinematics():
    def __init__(self):
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.right_wheel_pub = rospy.Publisher(
            '/control_right_wheel/command', Float32, queue_size=1)
        self.left_wheel_pub = rospy.Publisher(
            '/control_left_wheel/command', Float32, queue_size=1)
        self.cmd_vel = Twist()
        self.vl = Float32()
        self.vr = Float32()
        self.rpmL = Float32()
        self.rpmR = Float32()
        self.rate = rospy.Rate(10)

    def callback(self, msg):
        self.cmd_vel = msg

    def pub_wheel_vels(self):
        while not rospy.is_shutdown():
            self.convert_vels()
            self.left_wheel_pub.publish(self.rpmL)
            self.right_wheel_pub.publish(self.rpmR)
            self.rate.sleep()

    def convert_vels(self):
        L = 0.309  # //L is distance between wheels (wheel base)
        R = 0.0421  # //R is RADIUS     Diameter is 0.12
        self.vl = self.cmd_vel.linear.x - (self.cmd_vel.angular.z*(L/2))
        self.vr = self.cmd_vel.linear.x + (self.cmd_vel.angular.z*(L/2))
        self.rpmL = (self.vl*60)/(2*pi*R)
        self.rpmR = (self.vr*60)/(2*pi*R)

        if self.rpmL >= 8:
            self.rpmL = 8

        if self.rpmR >= 8:
            self.rpmR = 8

        if self.rpmL <= -8:
            self.rpmL = -8

        if self.rpmR <= -8:
            self.rpmR = -8


if __name__ == '__main__':
    rospy.init_node('convert_vel_node', anonymous=True)
    diffdrive_object = Diff_drive_kinematics()
    diffdrive_object.pub_wheel_vels()