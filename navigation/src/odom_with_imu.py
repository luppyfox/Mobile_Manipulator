#! /usr/bin/env python3
import rospy
from math import pi, sin, cos, atan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64, Float32
import tf
from tf.broadcaster import TransformBroadcaster

class OdometryClass:
    def __init__(self):
            self.enc_l_sub = rospy.Subscriber('/Enc_L', Int64, self.callback_L)
            self.enc_r_sub = rospy.Subscriber('/Enc_R', Int64, self.callback_R)
            self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 1)
            self.test_pub = rospy.Publisher('/test_odom', Float32, queue_size = 1)

            self.enc_r_sub = rospy.Subscriber('/yaw_imu', Float32, self.callback_yaw)
            
            # self.th_pub = rospy.Publisher('/th', Float32, queue_size = 1) #For testing
            # self.th2_pub = rospy.Publisher('/th2', Float32, queue_size = 1) #For testing
            self.odom = Odometry()
            self.rate = rospy.Rate(200)
            self.odom_broadcaster = TransformBroadcaster()
            self.lastL_ticks = 0
            self.lastR_ticks = 0
            self.currentL_ticks = 0
            self.currentR_ticks = 0
            self.current_time = rospy.Time.now()
            self.last_time = rospy.Time.now()
            self.L = 0.309 #distance between robot wheels
            self.R = 0.0421 #radius of wheel unit m
            self.N = 20181 #total pulse per round
            self.x = 0
            self.y = 0
            self.theta = 0.0
            self.prev_yaw_data = 0.0
            self.yaw_data = 0.0
            self.dir_yaw = 0
            self.delta_th = 0.0
            self.updatePose()

    def callback_L(self, msg):
        self.currentL_ticks = msg.data
        if self.lastL_ticks == 0:
            self.lastL_ticks == msg.data

    def callback_R(self, msg):
        self.currentR_ticks = msg.data
        if  self.lastR_ticks == 0:
            self.lastR_ticks = msg.data
    
    def callback_yaw(self, msg):
        self.yaw_data = msg.data

    def calculate_rotation(self, prev_angle, current_angle):
    # Calculate the shortest rotation from prev_angle to current_angle
        difference = current_angle - prev_angle
        if difference > 180:
            difference -= 360
        elif difference < -180:
            difference += 360
        return difference
        
    def updatePose(self):
        prev_theta = 0.0
        rate_2 = rospy.Rate(0.2)
        rate_2.sleep()
        while (self.yaw_data == 0.0):
            self.prev_yaw_data = self.yaw_data
        self.prev_yaw_data = self.yaw_data             
      
        while not rospy.is_shutdown():
            delta_l = self.currentL_ticks - self.lastL_ticks
            delta_r = self.currentR_ticks - self.lastR_ticks
            d_l = 2 * pi * self.R * (delta_l / self.N)
            d_r = 2 * pi * self.R * (delta_r / self.N)
            
            self.lastL_ticks = self.currentL_ticks
            self.lastR_ticks = self.currentR_ticks
            
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()

            th = (d_r - d_l) / (self.L)
            
            dc = (d_r + d_l) / 2
            v = dc / dt
            w = th / dt 

            delta_x = dc * cos(self.delta_th)
            delta_y = dc * sin(self.delta_th)

            self.x += delta_x
            self.y += delta_y
            self.delta_th += th

            rotation = self.calculate_rotation(self.prev_yaw_data, self.yaw_data)
            self.theta += rotation
            self.prev_yaw_data = self.yaw_data
            # rospy.loginfo("----------------------------")
            # rospy.loginfo("theta %s" , self.theta)
            # rospy.loginfo("current %s" , self.yaw_data)
            # rospy.loginfo("prev_yaw_data %s" , self.prev_yaw_data)
            # rospy.loginfo("----------------------------")
            self.test_pub.publish(self.theta)

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, (self.theta * pi / 180 ))

            self.odom_broadcaster.sendTransform((self.x, self.y, 0), odom_quat, self.current_time, "base_footprint", "odom")
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"

            odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))

            odom.child_frame_id = "base_footprint"
            odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))
            
            self.odom_pub.publish(odom)

            self.last_time = self.current_time
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('pub_odom')
    oc = OdometryClass() 
    rospy.spin()


# #! /usr/bin/env python3
# import rospy
# from math import pi, sin, cos
# from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
# from std_msgs.msg import Int64
# from nav_msgs.msg import Odometry

# import tf
# from tf.broadcaster import TransformBroadcaster


# class OdometryClass:

#     def __init__(self):
#         self.left_ticks_sub = rospy.Subscriber(
#             '/Enc_L', Int64, self.getTicks_L)
#         self.right_ticks_sub = rospy.Subscriber(
#             '/Enc_R', Int64, self.getTicks_R)

#         self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
#         self.odom_broadcaster = TransformBroadcaster()
#         self.odom = Odometry()
#         self.rate = rospy.Rate(200)
#         self.lastLeftTicks = 0
#         self.lastRightTicks = 0
#         self.currentLeftTicks = 0
#         self.currentRightTicks = 0
#         self.current_time = rospy.Time.now()
#         self.last_time = rospy.Time.now()

#         self.L = 0.308
#         self.R = 0.0421
#         self.N = 20181

#         self.x = 0.0
#         self.y = 0.0
#         self.th = 0.0

#         rospy.loginfo("Odom Start!!!")
#         self.updatePose()

#     def getTicks_L(self, msg):
#         self.currentLeftTicks = msg.data

#         if self.lastLeftTicks == 0:
#             self.lastLeftTicks = msg.data

#     def getTicks_R(self, msg):
#         self.currentRightTicks = msg.data

#         if self.lastRightTicks == 0:
#             self.lastRightTicks = msg.data

#     def updatePose(self):

#         while not rospy.is_shutdown():

#             delta_l = self.currentLeftTicks - self.lastLeftTicks
#             delta_r = self.currentRightTicks - self.lastRightTicks
#             d_l = 2 * pi * self.R * (delta_l / self.N)
#             d_r = 2 * pi * self.R * (delta_r / self.N)

#             self.lastLeftTicks = self.currentLeftTicks
#             self.lastRightTicks = self.currentRightTicks
#             self.current_time = rospy.Time.now()
#             dt = (self.current_time - self.last_time).to_sec()

#             dc = (d_r + d_l)/2
#             th = (d_r - d_l)/self.L

#             v = dc / dt
#             w = th / dt

#             # compute odometry in a typical way given the velocities of the robot
#             delta_x = dc * cos(self.th)
#             delta_y = dc * sin(self.th)
#             delta_th = th

#             self.x += delta_x
#             self.y += delta_y
#             self.th += delta_th

#             # since all odometry is 6DOF we'll need a quaternion created from yaw
#             odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

#             # first, we'll publish the transform over tf
#             # self.odom_broadcaster.sendTransform(
#             #    (self.x, self.y, 0),
#             #    odom_quat,
#             #     self.current_time,
#             #     "base_footprint",
#             #     "odom"
#             # )

#             # next, we'll publish the odometry message over ROS
#             odom = Odometry()
#             odom.header.stamp = self.current_time
#             odom.header.frame_id = "odom"

#             # set the position
#             odom.pose.pose = Pose(
#                 Point(self.x, self.y, 0.), Quaternion(*odom_quat))

#             # set the velocity
#             odom.child_frame_id = "base_footprint"
#             odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

#             # publish the message
#             self.odom_pub.publish(odom)

#             self.last_time = self.current_time
#             self.rate.sleep()


# if __name__ == "__main__":
#     rospy.init_node('odom_node')
#     oc = OdometryClass()
#     rospy.spin()
