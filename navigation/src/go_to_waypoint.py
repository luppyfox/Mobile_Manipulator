#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int64
import math

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # Publishers for controlling wheel speeds
        self.right_wheel_pub = rospy.Publisher('/control_right_wheel/command', Float32, queue_size=1)
        self.left_wheel_pub = rospy.Publisher('/control_left_wheel/command', Float32, queue_size=1)
        self.sub_x = rospy.Subscriber('/odom_x', Float32, self.callback_x)
        self.sub_y = rospy.Subscriber('/odom_y', Float32, self.callback_y)
        self.sub_yaw = rospy.Subscriber('/odom_yaw', Float32, self.callback_yaw)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0 # Yaw in degrees
        
        # Movement parameters
        self.tolerance_linear = 0.01 # meters
        self.tolerance_angle = 2 # degrees
        self.min_speed_rpm = 3.0
        self.max_speed_rpm = 5.0

    def callback_x(self, msg):
        self.x = msg.data

    def callback_y(self, msg):
        self.y = msg.data
    
    def callback_yaw(self, msg):
        self.theta = msg.data

    # def update_state(self, x, y, theta):
    #     self.x = x
    #     self.y = y
    #     self.theta = theta

    def rad_to_deg(self, rad):
        return rad * (180.0 / math.pi)

    def deg_to_rad(self, deg):
        return deg * (math.pi / 180.0)

    def rotate_to_yaw(self, target_yaw):
        error_yaw = target_yaw - self.theta
        while abs(error_yaw) > self.tolerance_angle:
            speed = max(min(abs(error_yaw), self.max_speed_rpm), self.min_speed_rpm)
            if error_yaw > 0:
                # Rotate clockwise
                self.right_wheel_pub.publish(Float32(-speed))
                self.left_wheel_pub.publish(Float32(speed))
            else:
                # Rotate counter-clockwise
                self.right_wheel_pub.publish(Float32(speed))
                self.left_wheel_pub.publish(Float32(-speed))
            
            # Update your robot's current yaw here, and recalculate error_yaw
            # This is just a placeholder for demonstration
            # error_yaw = updated_yaw - self.theta

    def move_to_point(self, target_x, target_y):
        error_x = target_x - self.x
        error_y = target_y - self.y
        target_yaw = self.rad_to_deg(math.atan2(error_y, error_x))
        
        # First, rotate to align with the target direction
        self.rotate_to_yaw(target_yaw)

        # Then move towards the target point
        distance = math.sqrt(error_x**2 + error_y**2)
        while distance > self.tolerance_linear:
            speed = max(min(distance, self.max_speed_rpm), self.min_speed_rpm)
            # Move forward
            self.right_wheel_pub.publish(Float32(speed))
            self.left_wheel_pub.publish(Float32(speed))
            
            # Update your robot's current position here, and recalculate distance
            # This is just a placeholder for demonstration
            # distance = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)

    def run(self, waypoints):
        for point in waypoints:
            self.move_to_point(point[0], point[1])
            self.rotate_to_yaw(point[2])
            rospy.sleep(1) # Wait a bit between movements

if __name__ == '__main__':
    controller = RobotController()
    waypoints = [
        [1.0341, 0, 62.253],
        [1.4735, -1.4903, 27.9916],
        [1.8948, -1.3096, 84.7814],
        [1.9737, -0.3926, 90]
    ]
    try:
        controller.run(waypoints)
    except rospy.ROSInterruptException:
        rospy.loginfo("Failed to run this code")
        pass
