#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def callback(msg):
    rospy.loginfo('serveo angle : {%f}', msg.data)

def main():
    rospy.init_node('Servo')
    rospy.Subscriber("/Servo_Angle", Float32, callback)
    rospy.spin()



if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
