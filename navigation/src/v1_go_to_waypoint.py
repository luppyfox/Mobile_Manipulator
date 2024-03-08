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
        self.tolerance_linear = 0.03 # meters
        self.tolerance_angle = 2 # degrees
        self.min_speed_rpm = 1.0
        self.max_speed_rpm = 2.0

        self.point = []
        self.waypoints = []

        self.rate = rospy.Rate(160)

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
        
    def ros_shutdown(self):
        if rospy.is_shutdown():
            rospy.signal_shutdown("Terminate")
            exit()

    def rad_to_deg(self, rad):
        return rad * (180.0 / math.pi)

    def deg_to_rad(self, deg):
        return deg * (math.pi / 180.0)

    def rotate_to_yaw(self, target_yaw):
        
        error_yaw = target_yaw - self.theta
        if error_yaw > 180:
            error_yaw -= 360
        elif error_yaw < -180:
            error_yaw += 360

        while abs(error_yaw) > self.tolerance_angle:

            error_yaw = target_yaw - self.theta
            if error_yaw > 180:
                error_yaw -= 360
            elif error_yaw < -180:
                error_yaw += 360
        
            speed = max(min(abs(error_yaw*5), self.max_speed_rpm), self.min_speed_rpm)
            if error_yaw > 0:
                # Rotate clockwise
                self.right_wheel_pub.publish(Float32((speed-0.1)))
                self.left_wheel_pub.publish(Float32(-speed))
            else:
                # Rotate counter-clockwise
                self.right_wheel_pub.publish(Float32(-(speed-0.1)))
                self.left_wheel_pub.publish(Float32(speed))
            rospy.loginfo("    error_yaw is: %s" , error_yaw)
            rospy.loginfo("    right is: %s" , speed)
            rospy.loginfo("    left is: %s" , speed)
            self.ros_shutdown()
            # rospy.sleep(0.00001) # Wait a bit between movements
            self.rate.sleep()
            
            # Update your robot's current yaw here, and recalculate error_yaw
            # This is just a placeholder for demonstration
            # error_yaw = updated_yaw - self.theta

    def move_to_point(self, target_x, target_y):
        error_x = target_x - self.x
        error_y = target_y - self.y
        # target_yaw = self.rad_to_deg(math.atan2(error_y, error_x))

        # # First, rotate to align with the target direction
        # self.rotate_to_yaw(target_yaw)

        # Then move towards the target point
        distance = math.sqrt(error_x**2 + error_y**2)
        prev_error_x = error_x
        prev_error_y = error_y

        while distance >= self.tolerance_linear:
            error_x = target_x - self.x
            error_y = target_y - self.y
            distance = math.sqrt(error_x**2 + error_y**2)
            
            # Determine if the robot has passed the target by checking the sign change of the errors
            # passed_target = (error_x * prev_error_x < 0) or (error_y * prev_error_y < 0)

            speed = max(min(distance*5, self.max_speed_rpm), self.min_speed_rpm)
            self.right_wheel_pub.publish(Float32((speed-0.1)))
            self.left_wheel_pub.publish(Float32(speed))

            # # Adjust movement direction based on the current position relative to the target
            # if not passed_target and distance > self.tolerance_linear:
            #     self.right_wheel_pub.publish(Float32(speed))
            #     self.left_wheel_pub.publish(Float32(speed))
            # else:
                
            #     while distance > self.tolerance_linear:
            #         error_x = target_x - self.x
            #         error_y = target_y - self.y
            #         distance = math.sqrt(error_x**2 + error_y**2)
            #         speed = max(min(distance*5, self.max_speed_rpm), self.min_speed_rpm)
            #         # Reverse the movement if the robot has passed the target
            #         self.right_wheel_pub.publish(Float32(-speed))
            #         self.left_wheel_pub.publish(Float32(-speed))
            #         self.ros_shutdown()
            #         rospy.sleep(0.00001)
            #         rospy.loginfo("----------------------------------------------------------------")

            prev_error_x = error_x
            prev_error_y = error_y
            rospy.loginfo("    distance is: %s", distance)
            rospy.loginfo("    right is: %s", speed)
            rospy.loginfo("    left is: %s", speed)
            self.ros_shutdown()
            # rospy.sleep(0.00001) # Wait a bit between movements
            self.rate.sleep()
            # Update your robot's current position here, and recalculate distance
            # This is just a placeholder for demonstration
            # distance = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
        rospy.loginfo("----------------------------------------------------------------")

    def run(self, waypoints):
        self.waypoints = waypoints
        for self.point in self.waypoints:
            rospy.loginfo("=====================================")
            rospy.loginfo("Move forward")
            rospy.loginfo("    Goal is: %s" , self.point)
            self.move_to_point(self.point[0], self.point[1])
            rospy.loginfo("=====================================")
            rospy.loginfo("Rotate")
            rospy.loginfo("    Goal is: %s" , self.point)
            self.rotate_to_yaw(self.point[2])
            rospy.sleep(1) # Wait a bit between movements
            rospy.loginfo("=====================================")

if __name__ == '__main__':
    controller = RobotController()
    waypoints = [
        [1.0341, 0, -62.2538],
        [1.4735, -1.4903, -57.9916],
        [1.8948, -1.3096, 84.7814],
        [1.9737, -0.3926, 90]
    ]
    try:
        controller.run(waypoints)
    except rospy.ROSInterruptException:
        rospy.loginfo("Failed to run this code")
        pass
