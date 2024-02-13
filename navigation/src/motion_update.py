#! /usr/bin/env python3
import rospy
from std_srvs.srv import Empty

def call_service():
    rate = rospy.Rate(0.2) # 1 times per 5 second
    try:
        rospy.wait_for_service('/request_nomotion_update')
        reset_service = rospy.ServiceProxy('request_nomotion_update', Empty)
    except:
        print("Call service fail")
    
    rate.sleep()
    rospy.spin()

if __name__=="__main__":
    rospy.init_node("call_update")
    call_service()