#! /usr/bin/env python3
from math import pi
import rospy
from std_msgs.msg import Int64, Float32
import matplotlib.pyplot as plt
import pyFileInteraction as pfi

class Data_Test():
    def __init__(self):
        self.vl_sub = rospy.Subscriber('/topicV_L', Float32, self.callback_L)
        self.vr_sub = rospy.Subscriber('/topicV_R', Float32, self.callback_R)
        self.right_wheel_pub = rospy.Publisher(
            '/control_right_wheel/command', Float32, queue_size=1)
        self.left_wheel_pub = rospy.Publisher(
            '/control_left_wheel/command', Float32, queue_size=1)
        self.vl = 0
        self.vr = 0
        self.rpmL = 8
        self.rpmR = 8
        self.start_time = rospy.Time.now()
        self.data_time = []
        self.data_vl = []
        self.data_vr = []
        self.data_all = []
        self.rate = rospy.Rate(50)
        

    def callback_L(self, msg):
        self.vl = msg.data

    def callback_R(self, msg):
        self.vr = msg.data

    def ploting(self):
        while not rospy.is_shutdown():
            # self.left_wheel_pub.publish(self.rpmL)
            self.right_wheel_pub.publish(self.rpmR)

            self.data_time.append((rospy.Time.now() - self.start_time).to_sec())
            self.data_vl.append(self.vl)
            self.data_vr.append(self.vr)
            self.data_all.append([(rospy.Time.now() - self.start_time).to_sec(), self.vl, self.vr])

            self.rate.sleep()

        # self.left_wheel_pub.publish(0)
        # self.right_wheel_pub.publish(0)
        plt.plot(self.data_time, self.data_vl, self.data_time, self.data_vr)
        plt.title('PID test')
        plt.show()

        pfi.write_csv(self.data_all, "/home/phat/catkin_ws/src/Mobile_Manipulator/navigation/csv/test_pid.csv")
        


if __name__ == '__main__':
    rospy.init_node('test_data_node', anonymous=True)
    T = Data_Test()
    T.ploting()