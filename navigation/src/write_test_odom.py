#! /usr/bin/env python3
from math import pi, degrees
import rospy
from std_msgs.msg import Int64, Float32
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import pyFileInteraction as pfi
import tf
from tf.transformations import euler_from_quaternion

class Data_Test():
    def __init__(self):
        self.vl_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        self.right_wheel_pub = rospy.Publisher(
            '/control_right_wheel/command', Float32, queue_size=1)
        self.left_wheel_pub = rospy.Publisher(
            '/control_left_wheel/command', Float32, queue_size=1)

        self.rpmL = 8
        self.rpmR = 8
        self.x = 0
        self.y = 0
        self.th = 0
        self.odom = Odometry()
        self.start_time = rospy.Time.now()
        self.rate = rospy.Rate(10)

        self.data_time = []
        self.data_x = []
        self.data_y = []
        self.data_th = []
        self.data_all = []
    
    def to_positive_angle(self, th):
        while True:
            if th < 0:
                th += 360
            if th > 0:
                ans = th % 360
                return ans
                break
        

    def callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.q1 = data.pose.pose.orientation.x
        self.q2 = data.pose.pose.orientation.y
        self.q3 = data.pose.pose.orientation.z
        self.q4 = data.pose.pose.orientation.w
        self.q = (self.q1, self.q2, self.q3, self.q4)
        self.e = euler_from_quaternion(self.q)
        self.th = degrees(self.e[2])
        self.th = self.to_positive_angle(self.th)

    def ploting(self):
        while not rospy.is_shutdown():
            if (self.x == 0):
                rospy.loginfo("Can't recive odom value")
            if (self.x >= 1.0):
                break
            self.left_wheel_pub.publish(self.rpmL)
            self.right_wheel_pub.publish(self.rpmR)

            self.data_time.append((rospy.Time.now() - self.start_time).to_sec())
            self.data_x.append(self.x)
            self.data_y.append(self.y)
            self.data_th.append(self.th)
            self.data_all.append([(rospy.Time.now() - self.start_time).to_sec(), self.data_x, self.data_y, self.data_th])

            self.rate.sleep()

        self.left_wheel_pub.publish(0)
        self.right_wheel_pub.publish(0) 
        rospy.loginfo("Finish")
        # plt.plot(self.data_time, self.data_vl, self.data_time, self.data_vr)
        # plt.title('PID test')
        # plt.show()

        pfi.write_csv(self.data_all, "/home/phat/catkin_ws/src/Mobile_Manipulator/navigation/csv/test_odom.csv")       


if __name__ == '__main__':
    rospy.init_node('test_data_node', anonymous=True)
    T = Data_Test()
    T.ploting()