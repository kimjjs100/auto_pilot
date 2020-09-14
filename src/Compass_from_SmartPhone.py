#!/usr/bin/env python
from sensor_msgs.msg import MagneticField 
from msg_package.msg import HeadingAngle
from Android_SensorManager import getRotationMatrix, getOrientation
import rospy
import math

class Filter:
    def __init__(self):
        self.weight = 0.2

    def kalman_filter(self, previous, present):        
        result = self.weight * previous + (1 - self.weight) * present
        return result

class SmartPhone_Magnetometer:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None 

        self.mAzimuth = None
        self.mPitch = None
        self.mRoll = None

        self.filter_x = None
        self.filter_y = None
        self.filter_z = None
        
        self.Compass_pub = rospy.Publisher('/HeadingAngle_RealTime', HeadingAngle, queue_size=10)
        rospy.Subscriber("/phone1/android/magnetic_field", MagneticField, self.mag_callback)   

    def mag_callback(self, MagneticField):
        # unit : [uT = 10^-6 T]
        self.x = MagneticField.magnetic_field.x * 10**6
        self.y = MagneticField.magnetic_field.y * 10**6
        self.z = MagneticField.magnetic_field.z * 10**6

    def HeadingAngle(self):        
        if self.x is None:
            return  
        else: 
            if self.filter_x is None:
                self.filter_x = self.x
                self.filter_y = self.y
                self.filter_z = self.z
                return

            previous_x = self.filter_x
            previous_y = self.filter_y
            previous_z = self.filter_z
                    
            F = Filter()

            self.filter_x = F.kalman_filter(previous_x, self.x)
            self.filter_y = F.kalman_filter(previous_y, self.y)
            self.filter_z = F.kalman_filter(previous_z, self.z)
            
            gravity = [0, 0, -9.8]
            geomagnetic = [self.filter_x, self.filter_y, self.filter_z]  

            R = [0] * 9
            orientation = [0] * 3

            success = getRotationMatrix(R, None, gravity, geomagnetic)
            if success is True:
                getOrientation(R, orientation)
                self.mAzimuth = orientation[0] * 180 / math.pi
                self.mPitch = orientation[1] * 180 / math.pi
                self.mRoll = orientation[2] * 180 / math.pi

    def round_Headinganlge_pub(self): 
        if self.mAzimuth is None:
            return
        mAzimuth = self.mAzimuth * 2
        mAzimuth = round(mAzimuth, -1)
        mAzimuth = mAzimuth / 2

        self.Compass_pub.publish(mAzimuth)
        rospy.loginfo(mAzimuth)
                
    def HeadingAngle_pub(self):        
        self.Compass_pub.publish(self.mAzimuth)
        rospy.loginfo(self.mAzimuth)


def main():
    gravity = [0, 0, -9.8]
    rospy.init_node('Compass', anonymous = False)
    rate = rospy.Rate(5)

    compass = SmartPhone_Magnetometer()   
    while not rospy.is_shutdown():
        compass.HeadingAngle()
        #compass.HeadingAngle_pub()
        compass.round_Headinganlge_pub()
        rate.sleep()
    rospy.spin()


if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
