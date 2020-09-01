#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16


def callback(data):
    msg_input = data.data
    rospy.loginfo('Thruset input : {%i}', msg_input) 
    rospy.loginfo("")   

def main():
    rospy.init_node('Thruster', anonymous=False)
    rospy.Subscriber('/thruster', Int16, callback)
    rospy.spin()



if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass