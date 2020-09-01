#!/usr/bin/env python
#from magnetometer_pkg import lis3mdl
from msg_package.msg import HeadingAngle
import rospy
import math
import time

#magnet = lis3mdl.LIS3MDL()
#magnet.enableLIS()

class Filter:
    def __init__(self):
        self.weight = rospy.get_param("weight")

    def kalman_x(self, previous_x, x):
        result_x = self.weight * previous_x + (1 - self.weight) * x
        return(result_x)

    def kalman_y(self, previous_y, y):
        result_y = self.weight * previous_y + (1 - self.weight) * y
        return(result_y)


class Compass:
    def __init__(self):
        self.calibration_offsetX = rospy.get_param("calibration_offsetX")
        self.calibration_offsetY = rospy.get_param("calibration_offsetY")
        self.calibration_scaleFactorX = rospy.get_param("calibration_scaleFactorX")
        self.calibration_scaleFactorY = rospy.get_param("calibration_scaleFactorY")

        self.filter_x = 0
        self.filter_y = 0
        
        self.Compass_pub = rospy.Publisher('/HeadingAngle_RealTime', HeadingAngle, queue_size=10)
    
 
    def HeadingAngle(self):
        #data_raw = magnet.getMagnetometerRaw()        
        data_raw = [time.clock() , time.clock()  ]

         
        previous_x = self.filter_x
        previous_y = self.filter_y

        
        F = Filter()

        self.filter_x = F.kalman_x(previous_x, data_raw[0])
        self.filter_y = F.kalman_y(previous_y, data_raw[1])

        x = self.calibration_scaleFactorX * (self.filter_x + self.calibration_offsetX)
        y = self.calibration_scaleFactorY * (self.filter_y + self.calibration_offsetY)

        heading = math.atan2(y, x) * 180/math.pi

        if (heading < 0) :
            heading = 360 + heading

        self.Compass_pub.publish(heading)
        #rospy.loginfo("Heading angle : {%f}", heading)


def main():   
    rospy.init_node('Compass', anonymous = False)
    rate = rospy.Rate(10)

    compass = Compass()   
    while not rospy.is_shutdown():
        compass.HeadingAngle()
        rate.sleep()

if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
        

