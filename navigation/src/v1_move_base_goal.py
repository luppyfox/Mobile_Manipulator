#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, Pose2D
from tf.transformations import quaternion_from_euler

class MoveBaseSeq:
    def __init__(self):
        rospy.init_node('move_base_sequence')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Subscriber to pose2D
        self.pose_sub = rospy.Subscriber('pose2D', Pose2D, self.pose_callback)

        self.client.wait_for_server()

    def pose_callback(self, data):
        rospy.loginfo("----------------------------")
        rospy.loginfo("x     : %s" , data.x)
        rospy.loginfo("y     : %s" , data.y)
        rospy.loginfo("theta : %s" , data.theta)
        rospy.loginfo("----------------------------")

    def send_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # Convert the Euler angle to a quaternion.
        q = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation = Quaternion(*q)

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
        else:
            return self.client.get_result()

if __name__ == '__main__':
    try:
        mover = MoveBaseSeq()
        # Send the first goal
        mover.send_goal(1, 0, 0)
        mover.pose_callback()
        rospy.loginfo("First goal completed")
        # Send the second goal
        mover.send_goal(2, 1, 0)
        mover.pose_callback()
        rospy.loginfo("Second goal completed")
    except rospy.ROSInterruptException:
        pass