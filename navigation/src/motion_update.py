#! /usr/bin/env python3
import rospy
from std_srvs.srv import Empty

def call_service():
    rate = rospy.Rate(0.2) # 1 times per 5 second
    try:
        rospy.wait_for_service('request_nomotion_update')
        reset_service = rospy.ServiceProxy('request_nomotion_update', Empty)
        rospy.loginfo("test2")
    except:
        rospy.loginfo("Call service fail")
        # rospy.loginfo("test")
    
    rate.sleep()
    # rospy.spin()

if __name__=="__main__":
    rospy.init_node("call_update", anonymous=True)
    while not rospy.is_shutdown():
        call_service()
        # rospy.loginfo("test3")
        # rospy.spin()