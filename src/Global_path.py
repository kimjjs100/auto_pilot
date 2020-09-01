#!/usr/bin/env python
import math
import rospy
from msg_package.msg import GPS
from msg_package.msg import HeadingAngle
from msg_package.msg import ThrustAngle
from std_msgs.msg import Float32        # for `Thruster_Output`

class PID:
    """PID controller."""

    def __init__(self, Kp, Ki, Kd):
        # Gains for each term
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Corrections (outputs)
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0

        self.previous_error = 0.0

    def Update(self, error):
        # dt = random specified value
        dt = 0.1 
        de = error - self.previous_error

        self.Cp = error
        self.Ci += error * dt
        self.Cd = de / dt

        self.previous_error = error

        return (
            (self.Kp * self.Cp)    # proportional term
            + (self.Ki * self.Ci)  # integral term
            + (self.Kd * self.Cd)  # derivative term
        )

class GlobalPath:
    def __init__(self):
        self.GPS_x = 0.0
        self.GPS_y = 0.0
        self.psi = 0.0
        self.i = 0 # for `Waypoints List` Index
        self.error_Distance = 10.0 # If 'error_Distance =0', the start point of the waypoint will be change
        self.target_angleAngle = 0.0
        #self.error_N = 0.0
        rospy.Subscriber("/GPS_RealTime", GPS, self.gps_callback)
        rospy.Subscriber("/HeadingAngle_RealTime", HeadingAngle, self.HeadingAngle_callback)

        self.thruster_pub = rospy.Publisher("Global_Angle", ThrustAngle, queue_size=10 )
        self.error_pub = rospy.Publisher("error_Distance", Float32, queue_size=10)

    # Get from `GPS node`
    def gps_callback(self, msg):
        self.GPS_x = msg.x
        self.GPS_y = msg.y

    def HeadingAngle_callback(self, msg):
        self.psi = msg.headingAngle    # Heading Angle

    # Unpack of `waypoints list`
    def waypoint_step(self, waypoints, error_Distance, tolerance): 
        if self.i + 1 != len(waypoints):
            if error_Distance > tolerance:
                waypoint = waypoints[self.i]
            elif error_Distance <= tolerance:
                waypoint = waypoints[self.i + 1]
                self.i = self.i + 1

        elif self.i + 1 == len(waypoints):
            waypoint = waypoints[self.i]

        return waypoint

    def target_angle_output(self, target_angle):
        '''
        `self.target_angle_output` changes Servo Input from 23.2845 degrees to 20 degrees.
        ex1) 95 deg -> 90 deg / -180 deg -> -90 deg
        ex2) 48.1572345 deg -> 50 deg / 22.945179 deg -> 20 deg
        '''
        if target_angle >= 90:
            target_angle = 90
            return target_angle
        elif target_angle > -90 and target_angle < 90:
            target_angle = target_angle * 2
            target_angle = round(target_angle, -1)
            target_angle = target_angle / 2
            return target_angle
        elif target_angle <= -90:
            target_angle = -90
            return target_angle

    # LOS Guidance + PID Control     
    def LOS_Guidance(self, waypoint_x, waypoint_y): 
        # Advance direction angle error

        if self.GPS_x == 0.0 and self.GPS_y == 0.0 and self.psi == 0.0:
            return
        
        else:   
            error_x = waypoint_x - self.GPS_x
            error_y = waypoint_y - self.GPS_y

            theta =  math.atan2(error_x, error_y) *180 / math.pi
            '''
            # theta : angle of waypoint deriection in Earth coordinate system
            if theta <0: 
                # compensate the angle representation range from [-pi, pi] to [0, 2*pi]
                theta = theta +2*math.pi
            '''
            rospy.loginfo(' theta = %d, psi = %d', theta, self.psi)
            error_N = -theta - self.psi
            # error_N : error of y between the target and the ship
            # if error_N ~= 0 : True direction
            '''
            # compensate the angle representation range from [-pi, pi] to [0, 2*pi]
            if math.pi <= error_N and error_N < 2*math.pi:        
                error_N = error_N - 2*math.pi
            elif (-2)*math.pi < error_N and error_N < (-1)*math.pi:
                error_N = error_N + 2*math.pi
            '''
            self.error_Distance = math.sqrt(error_x*error_x + error_y*error_y)  
            rospy.loginfo('%f', self.error_Distance)  
            self.error_pub.publish(self.error_Distance)
            # error_Distance : the distance left to waypoint
            '''
            gains = rospy.get_param('PID_gains')
            Kp, Ki, Kd = gains['Kp'], gains['Ki'], gains['Kd']

            pid = PID(Kp, Ki, Kd)  # PID (Kp, Ki, Kd)
            correction = pid.Update(error_N) * 180 / math.pi      
            self.target_angleAngle = self.target_angle_output(correction)
            '''

            self.target_angleAngle = error_N

            '''
            `self.target_angle_output` changes Servo Input from 23.2845 degrees to 20 degrees.
            '''

    def thrusterPublisher(self):
        if self.target_angleAngle == 0.0:
            return
        else:    
            self.thruster_pub.publish(self.target_angleAngle)  
                  


def main():
    rospy.init_node('Path_Planner', anonymous=False)
    rate = rospy.Rate(10) # 10 Hz renew

    waypoints = rospy.get_param("waypoint_List/waypoints") # Define `Waypoints List`
    tolerance = rospy.get_param("waypoint_distance_tolerance") # reach criterion

    PathPlan = GlobalPath()
    while not rospy.is_shutdown():
        waypoint = PathPlan.waypoint_step(waypoints, PathPlan.error_Distance, tolerance)
        waypoint_x = waypoint[0] # waypoint = (x, y)
        waypoint_y = waypoint[1]

        PathPlan.LOS_Guidance(waypoint_x, waypoint_y)
        PathPlan.thrusterPublisher()
        rate.sleep()        
    rospy.spin()

if __name__ =='__main__':
    main()
