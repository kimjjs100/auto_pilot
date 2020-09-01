#!/usr/bin/env python
from std_msgs.msg import Float32MultiArray
from lis3mdl import LIS3MDL
import rospy
import time
import threading

class Calibration_class:
    def __init__(self):
        self.x_scale = None
        self.y_scale = None
        self.z_scale = None

        self.offset_x = None
        self.offset_y = None
        self.offset_z = None

        self.min_x = None
        self.min_y = None
        self.min_z = None

        self.max_x = None
        self.max_y = None
        self.max_z = None

        self.calibration_pub = rospy.Publisher('/Calibration_prameter', Float32MultiArray, queue_size =8)        

    def calibration(self):
        SAMPLE_SIZE = 500

        magnetometer = LIS3MDL()
        magnetometer.enableLIS()

        ''' 
        key_listener = KeyListener()
        key_listener.start()
        '''
        ############################
        # Magnetometer Calibration #
        ############################

        mag_x_Raw =[ ]
        mag_y_Raw =[ ]
        mag_z_Raw =[ ]
    
        mag_x, mag_y, mag_z = magnetometer.getMagnetometerRaw()
        min_x = max_x = mag_x
        min_y = max_y = mag_y
        min_z = max_z = mag_z
    
        for _ in range(SAMPLE_SIZE):
            mag_x, mag_y, mag_z = magnetometer.getMagnetometerRaw()
            
            mag_x_Raw.append(mag_x)
            mag_y_Raw.append(mag_y)
            mag_z_Raw.append(mag_z)
    
            print(
                "Magnetometer: X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
                    mag_x, mag_y, mag_z
                )
            )
    
            min_x = min(self.min_x, mag_x)
            min_y = min(self.min_y, mag_y)
            min_z = min(self.min_z, mag_z)
    
            max_x = max(self.max_x, mag_x)
            max_y = max(self.max_y, mag_y)
            max_z = max(self.max_z, mag_z)

            
            offset_x = (max_x + min_x) / 2
            offset_y = (max_y + min_y) / 2
            offset_z = (max_z + min_z) / 2
    
            field_x = (max_x - min_x) / 2
            field_y = (max_y - min_y) / 2
            field_z = (max_z - min_z) / 2

            avg_rad = (field_x + field_y + field_z)
            avg_rad = avg_rad / 3
            
    
            print(
                "Hard Offset:  X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
                    offset_x, offset_y, offset_z
                )
            )
            print(
                "Field:        X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
                    field_x, field_y, field_z
                )
            )
            print("")
            time.sleep(0.01)

        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z

        self.min_x = min_x
        self.min_y = min_y
        self.min_z = min_z

        self.max_x = max_x
        self.max_y = max_y
        self.max_z = max_z

        self.x_scale = avg_rad / field_x
        self.y_scale = avg_rad / field_y
        self.z_scale = avg_rad / field_z
            

    def calibration_publisher(self):
        calibration = Float32MultiArray()        

        calibration.data = [0, 0, 0, 0]
        calibration.data[0] = self.x_scale
        calibration.data[1] = self.y_scale
        calibration.data[2] = self.offset_x
        calibration.data[3] = self.offset_y
        
        self.calibration_pub.publish(calibration)
        rospy.loginfo(calibration)


def main():
    rospy.init_node('Compass_Calibration', anonymous=False)
    calib = Calibration_class()

    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        calib.calibration()   
        calib.calibration_publisher()  
        
        rate.sleep()  
    rospy.spin()  



if __name__ == "__main__":
    main()