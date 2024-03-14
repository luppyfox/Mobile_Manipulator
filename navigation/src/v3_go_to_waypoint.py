#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import math

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')#, anonymous=True)
        self.pose_x, self.pose_y, self.pose_theta = 0.0, 0.0, 0.0  # Robot's current position (x, y) and orientation (theta in radians)
        self.right_wheel_publisher = rospy.Publisher('/control_right_wheel/command', Float32, queue_size=1)
        self.left_wheel_publisher = rospy.Publisher('/control_left_wheel/command', Float32, queue_size=1)
        self.pose_x_subscriber = rospy.Subscriber('/odom_x', Float32, self.update_pose_x)
        self.pose_y_subscriber = rospy.Subscriber('/odom_y', Float32, self.update_pose_y)
        self.pose_yaw_subscriber = rospy.Subscriber('/odom_yaw', Float32, self.update_pose_theta)

        self.max_speed = 3.0  # Maximum speed
        self.min_speed = 2.0  # Minimum speed

        self.tolerance_distance = 0.05  # Tolerance for reaching a point (meters)
        self.tolerance_orientation = math.radians(2)  # Tolerance for orientation (radians)

        self.navigation_points = [[1.0341, 0, -62.253], [1.4735, -1.4903, -57.9916], 
                                  [1.8948, -1.3096, 84.7814], [1.9737, -0.3926, 90]]
        
        self.rate = rospy.Rate(170)
    
    def update_pose_x(self, msg):
        self.pose_x = msg.data

    def update_pose_y(self, msg):
        self.pose_y = msg.data
    
    def update_pose_theta(self, msg):
        # Convert from degrees to radians for internal calculations
        self.pose_theta = math.radians(msg.data)

    def move_to_point(self, target):
        target_x, target_y, target_yaw_deg = target
        rospy.loginfo(f"Moving towards point: {target}")
        while not rospy.is_shutdown():
            dx = target_x - self.pose_x
            dy = target_y - self.pose_y
            distance = math.sqrt(dx**2 + dy**2)
            if distance < self.tolerance_distance:
                rospy.loginfo("Reached target point within tolerance.")
                break

            speed = self.calculate_speed(distance)
            angle_to_target = math.atan2(dy, dx)
            angle_diff_rad = self.normalize_angle(angle_to_target - self.pose_theta)

            right_speed, left_speed = self.calculate_wheel_speeds(speed, angle_diff_rad)
            self.right_wheel_publisher.publish(Float32(right_speed))
            self.left_wheel_publisher.publish(Float32(left_speed))

            rospy.loginfo("    angle_diff is: %s", angle_diff_rad)
            rospy.loginfo("    distance is: %s", distance)
            rospy.loginfo("    right is: %s", right_speed)
            rospy.loginfo("    left is: %s", left_speed)

            # rospy.sleep(0.1)  # Control loop frequency
            self.rate.sleep()

    def align_to_yaw(self, target_yaw_deg):
        rospy.loginfo(f"Aligning to yaw: {target_yaw_deg} degrees")
        target_yaw_rad = math.radians(target_yaw_deg)
        while not rospy.is_shutdown():
            yaw_diff_rad = self.normalize_angle(target_yaw_rad - self.pose_theta)
            if abs(yaw_diff_rad) < self.tolerance_orientation:
                rospy.loginfo("Orientation aligned within tolerance.")
                break

            wheel_speeds = self.calculate_rotation_speed(yaw_diff_rad)
            self.right_wheel_publisher.publish(Float32(wheel_speeds[0]))
            self.left_wheel_publisher.publish(Float32(wheel_speeds[1]))

            rospy.loginfo("    angle_diff is: %s", yaw_diff_rad)
            rospy.loginfo("    right is: %s", wheel_speeds[0])
            rospy.loginfo("    left is: %s", wheel_speeds[1])

            # rospy.sleep(0.1)  # Control loop frequency
            self.rate.sleep()

    def calculate_speed(self, distance):
        # Linear speed adjustment based on the distance to the target point
        return max(self.min_speed, min(self.max_speed, distance))

    def normalize_angle(self, angle):
        # Normalize angle to be within -pi and pi
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_wheel_speeds(self, speed, angle_diff_rad):
        # Simplistic differential drive robot model for demonstration
        # Adjust these calculations based on your robot's specific kinematics
        right_speed = speed + (angle_diff_rad * 2)
        left_speed = speed - (angle_diff_rad * 2)
        return right_speed, left_speed

    def calculate_rotation_speed(self, yaw_diff_rad):
        # Determine wheel speeds required for rotation
        if yaw_diff_rad > 0:
            # Rotate clockwise
            return self.min_speed, -self.min_speed
        else:
            # Rotate counter-clockwise
            return -self.min_speed, self.min_speed

    def run(self):
        for point in self.navigation_points:
            rospy.loginfo("=====================================")
            rospy.loginfo("Move forward")
            self.move_to_point(point)
            rospy.loginfo("=====================================")
            rospy.loginfo("Move rotate")
            self.align_to_yaw(point[2])  # Align to the yaw angle of the point

if __name__ == "__main__":
    robot_controller = RobotController()
    try:
        robot_controller.run()
    except rospy.ROSInterruptException:
        pass
