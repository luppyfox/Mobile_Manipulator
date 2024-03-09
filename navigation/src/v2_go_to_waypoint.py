#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import math

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.x, self.y, self.theta = 0.0, 0.0, 0.0  # Robot's current position and orientation
        self.right_wheel_pub = rospy.Publisher('/control_right_wheel/command', Float32, queue_size=1)
        self.left_wheel_pub = rospy.Publisher('/control_left_wheel/command', Float32, queue_size=1)
        self.sub_x = rospy.Subscriber('/odom_x', Float32, self.callback_x)
        self.sub_y = rospy.Subscriber('/odom_y', Float32, self.callback_y)
        self.sub_yaw = rospy.Subscriber('/odom_yaw', Float32, self.callback_yaw)

        self.max_speed = 5.0
        self.min_speed = 3.0

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0 # Yaw in degrees

        self.tolerance_linear = 0.05  # 1 cm
        self.tolerance_angle = 2 * math.pi / 180  # 2 degrees in radians
        self.points = [[1.0341, 0, -62.253], [1.4735, -1.4903, -57.9916], 
                       [1.8948, -1.3096, 84.7814], [1.9737, -0.3926, 90]]
        
        self.point_angle = 0.0
        self.count_angle = 0
        
        # self.rate = rospy.Rate(160)
        
    def callback_x(self, msg):
        self.x = msg.data

    def callback_y(self, msg):
        self.y = msg.data
    
    def callback_yaw(self, msg):
        self.theta = msg.data

    def rad_to_deg(self, rad):
        return rad * (180.0 / math.pi)

    def deg_to_rad(self, deg):
        return deg * (math.pi / 180.0)

    def move_to_point(self, target):
        target_x, target_y, _ = target
        rospy.loginfo(target_x)
        while not rospy.is_shutdown():
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.sqrt(dx**2 + dy**2)
            if distance < self.tolerance_linear:
                break  # Target reached within tolerance

            # Calculate desired speed based on distance
            speed = max(self.min_speed, min(self.max_speed, distance))

            # angle_to_target = math.atan2(dy, dx)
            # angle_diff = angle_to_target - self.rad_to_deg(self.theta)
            # if angle_diff > (math.pi):
            #     angle_diff -= (math.pi*2)
            # elif angle_diff < -(math.pi):
            #     angle_diff += (math.pi*2)
            if (self.count_angle == 0): self.point_angle = 0
            else: self.point_angle = self.points[self.count_angle-1][2]
            angle_diff = self.point_angle - self.theta
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360
            angle_diff = self.rad_to_deg(angle_diff)

            # Determine wheel speeds required to move towards target
            right_speed, left_speed = self.calculate_wheel_speeds(speed, angle_diff)
            self.right_wheel_pub.publish(right_speed)
            self.left_wheel_pub.publish(left_speed)
            rospy.loginfo("    angle_diff is: %s", angle_diff)
            rospy.loginfo("    distance is: %s", distance)
            rospy.loginfo("    right is: %s", right_speed)
            rospy.loginfo("    left is: %s", left_speed)
            rospy.sleep(0.1)  # Update frequency
            # self.rate.sleep()
        rospy.loginfo("----------------------------------------------------------------")

    def align_to_yaw(self, target_yaw):
        while not rospy.is_shutdown():
            yaw_diff = target_yaw - self.theta
            if yaw_diff > 180:
                yaw_diff -= 360
            elif yaw_diff < -180:
                yaw_diff += 360
            if abs(yaw_diff) < self.tolerance_angle:
                break  # Yaw aligned within tolerance

            # Calculate wheel speeds for rotation
            if yaw_diff > 0:
                # Rotate clockwise
                self.right_wheel_pub.publish(self.min_speed)
                self.left_wheel_pub.publish(-self.min_speed)
            else:
                # Rotate counter-clockwise
                self.right_wheel_pub.publish(-self.min_speed)
                self.left_wheel_pub.publish(self.min_speed)

            rospy.loginfo("    Error yaw is: %s", yaw_diff)
            # rospy.loginfo("    right is: %s", right_speed)
            # rospy.loginfo("    left is: %s", left_speed)
            rospy.sleep(0.1)  # Update frequency
            # self.rate.sleep()
        rospy.loginfo("----------------------------------------------------------------")

    def calculate_wheel_speeds(self, speed, angle_diff):
        # Simplified example; implement logic based on your robot's kinematics
        # This should return appropriate wheel speeds to move towards the target
        right_speed = speed + (angle_diff / 10)
        left_speed = speed - (angle_diff / 10)
        
        return max(self.min_speed, min(self.max_speed, right_speed)), max(self.min_speed, min(self.max_speed, left_speed))

    def run(self):
        for point in self.points:
            rospy.loginfo("=====================================")
            rospy.loginfo("Move forward")
            self.move_to_point(point)
            rospy.loginfo("=====================================")
            rospy.loginfo("Rotation")
            self.count_angle += 1
            self.align_to_yaw(point[2])  # Convert degrees to radians

if __name__ == "__main__":
    robot_controller = RobotController()
    try:
        robot_controller.run()
    except rospy.ROSInterruptException:
        pass
