#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray
from msg_package.msg import ThrustAngle

class Thrust:
    def __init__(self):
        self.target_angle = 0
        self.thrusterL = 0
        self.thrusterR = 0
        self.delay = 0
        self.thrusterL_pub = rospy.Publisher('/thrusterL', UInt16MultiArray, queue_size =8)
        self.thrusterR_pub = rospy.Publisher('/thrusterR', UInt16MultiArray, queue_size =8)
        
        rospy.Subscriber('/Local_Angle', ThrustAngle, self.local_callback)
        rospy.Subscriber('/Global_Angle', ThrustAngle, self.global_callback)
        rospy.Subscriber('/Idle_Angle', ThrustAngle, self.idle_callback)
        
    def local_callback(self, ThrustAngle):
        self.target_angle = ThrustAngle.theta
        if round(self.target_angle) > 360:
            self.target_angle = None

    def global_callback(self, ThrustAngle):
        self.target_angle = ThrustAngle.theta
        if round(self.target_angle) > 360:
            self.target_angle = None

    def idle_callback(self, ThrustAngle):
        self.target_angle = ThrustAngle.theta        
        if round(self.target_angle) > 360:
            self.target_angle = None
        
    def output(self):    

        rospy.loginfo(self.target_angle)

        if -5 <= self.target_angle <= 5:
            self.thrusterL = 1300 
            self.thrusterR = 1700
            self.delay = 125

        elif self.target_angle == 180 or self.target_angle == -180:
            self.thrusterL = 1600
            self.thrusterR = 1400
            self.delay = 1000

        elif 5 < self.target_angle <= 30:
            self.thrusterL = 1400
            self.thrusterR = 1400
            self.delay = 125
        elif 30 < self.target_angle <= 60:
            self.thrusterL = 1350
            self.thrusterR = 1350
            self.delay = 125
        elif 60 < self.target_angle <= 90:
            self.thrusterL = 1300
            self.thrusterR = 1300
            self.delay = 125
        elif 85 < self.target_angle < 180 :
            self.thrusterL = 1300
            self.thrusterR = 1300
            self.delay = 125

        elif -30 < self.target_angle <= -5:
            self.thrusterL = 1600
            self.thrusterR = 1600
            self.delay = 125
        elif -60 < self.target_angle <= -30:
            self.thrusterL = 1650
            self.thrusterR = 1650
            self.delay = 125
        elif -90 < self.target_angle <= -60:
            self.thrusterL = 1700
            self.thrusterR = 1700
            self.delay = 125
        elif -180< self.target_angle <= -85:
            self.thrusterL = 1700
            self.thrusterR = 1700
            self.delay = 125
        

    def thrustpublisher(self):
        thrusterL = UInt16MultiArray()
        thrusterR = UInt16MultiArray()

        thrusterL.data=[0]*2
        thrusterR.data=[0]*2

        thrusterL.data[0] = self.thrusterL
        thrusterL.data[1] = self.delay

        thrusterR.data[0] = self.thrusterR
        thrusterR.data[1] = self.delay

        self.thrusterL_pub.publish(thrusterL)
        self.thrusterR_pub.publish(thrusterR)

        #rospy.loginfo(thrusterL)
        #rospy.loginfo(delay)

def main():
    rospy.init_node('Thrust', anonymous = False)

    thrust = Thrust()
    rate = rospy.Rate(8)

    while not rospy.is_shutdown():
        thrust.output()
        thrust.thrustpublisher()
        rate.sleep()
    rospy.spin()
if __name__ == '__main__':
    main()
