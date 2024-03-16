#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler
from tf.broadcaster import TransformBroadcaster

class OdomPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher', anonymous=True)

        # Subscriber to pose2D
        self.pose_sub = rospy.Subscriber('pose2D', Pose2D, self.pose_callback)

        # Publisher to odom
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)

        self.odom_broadcaster = TransformBroadcaster()

        self.last_pose = Pose2D()
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(0.5)

    def pose_callback(self, data):
        if not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            
            # Calculate linear speed
            dx = data.x - self.last_pose.x
            dy = data.y - self.last_pose.y
            linear_speed = (dx ** 2 + dy ** 2) ** 0.5 / dt
            
            # Calculate angular speed
            dtheta = data.theta - self.last_pose.theta
            angular_speed = dtheta / dt
            
            # Prepare odometry message
            # odom_quat = tf.transformations.quaternion_from_euler(0, 0, data.theta)
            # self.odom_broadcaster.sendTransform((data.x, data.y, 0), odom_quat, self.current_time, "base_footprint", "odom")
            odom_msg = Odometry()
            odom_msg.header.stamp = self.current_time
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"

            # Set the position
            odom_msg.pose.pose.position.x = data.x
            odom_msg.pose.pose.position.y = data.y
            odom_msg.pose.pose.position.z = 0  # Assuming a flat surface

            # Convert theta (yaw) to a quaternion and set it
            q = quaternion_from_euler(0, 0, data.theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # Set linear and angular velocities
            odom_msg.twist.twist.linear.x = linear_speed
            odom_msg.twist.twist.angular.z = angular_speed

            # Publish the message
            self.odom_pub.publish(odom_msg)
            rospy.loginfo("----------------------------")
            rospy.loginfo("x     : %s" , data.x)
            rospy.loginfo("y     : %s" , data.y)
            rospy.loginfo("theta : %s" , data.theta)
            rospy.loginfo("----------------------------")

            # Update last pose and time
            self.last_pose = data
            self.last_time = self.current_time
            self.rate.sleep()
        else:
            rospy.signal_shutdown("Terminate")
            exit()

if __name__ == '__main__':
    try:
        odom_publisher = OdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
