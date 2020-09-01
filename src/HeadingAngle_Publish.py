#!/usr/bin/env python
from Android_SensorManager import getOrientation, getRotationMatrix
from msg_package.msg import HeadingAngle
import lis3mdl
import rospy
import math

magnet = lis3mdl.LIS3MDL()
magnet.enableLIS()

class Filter:
    def __init__(self):
        self.weight = 0.2

    def kalman_filter(self, previous, present):        
        result = self.weight * previous + (1 - self.weight) * present
        return result


class Compass:
    def __init__(self):
        self.mAzimuth = None
        self.mPitch = None
        self.mRoll = None
        
        self.filter_x = None
        self.filter_y = None
        self.filter_z = None
        
        self.Compass_pub = rospy.Publisher('/HeadingAngle_RealTime', HeadingAngle, queue_size=10)
    
 
    def HeadingAngle(self):
        data_raw = magnet.getMagnetometerRaw()     
        magnetic_x = data_raw[0] * (-1) * 10**(-2)
        magnetic_y = data_raw[1] * (-1) * 10**(-2)
        magnetic_z = data_raw[2] * (-1) * 10**(-2)

        if self.filter_x is None:
            self.filter_x = magnetic_x
            self.filter_y = magnetic_y
            self.filter_z = magnetic_z 
            return

        previous_x = self.filter_x
        previous_y = self.filter_y
        previous_z = self.filter_z
                
        F = Filter()
        
        self.filter_x = F.kalman_filter(previous_x, magnetic_x)
        self.filter_y = F.kalman_filter(previous_y, magnetic_y)
        self.filter_z = F.kalman_filter(previous_z, magnetic_z)
        
        gravity = [0, 0, -9.8]
        geomagnetic = [self.filter_x, self.filter_y, self.filter_z]  
        #print(self.filter_x, self.filter_y,  self.filter_z)

        R = [0] * 9
        orientation = [0] * 3

        success = getRotationMatrix(R, None, gravity, geomagnetic)
        if success is True:            
            getOrientation(R, orientation)            
            self.mAzimuth = orientation[0] * 180 / math.pi
            self.mPitch = orientation[1] * 180 / math.pi
            self.mRoll = orientation[2] * 180 / math.pi
                
    def HeadingAngle_pub(self):        
        self.Compass_pub.publish(self.mAzimuth)
        rospy.loginfo(self.mAzimuth)

        #self.Compass_pub.publish(heading)
        #rospy.loginfo("Heading angle : {%f}", heading)


def main():   
    rospy.init_node('Compass', anonymous = False)
    rate = rospy.Rate(10)

    compass = Compass()   
    while not rospy.is_shutdown():
        compass.HeadingAngle()
        compass.HeadingAngle_pub()
        rate.sleep()

if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
        

