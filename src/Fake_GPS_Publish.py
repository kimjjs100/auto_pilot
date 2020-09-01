#!/usr/bin/env python
import rospy
from msg_package.msg import GPS
import time
import math

def GPS_now():
    pub = rospy.Publisher('/GPS_RealTime', GPS, queue_size=10)
    rospy.init_node('GPS', anonymous = False)
    rate = rospy.Rate(10)

    msg = GPS()
    while not rospy.is_shutdown():
        msg.x = time.clock() 
        msg.y = time.clock()
        # rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ =='__main__':
    try:
        GPS_now()
    except rospy.ROSInitException:
        pass
        

