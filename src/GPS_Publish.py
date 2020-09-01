#!/usr/bin/env python
import rospy
from msg_package.msg import GPS
from functools import reduce
import time
import serial
import operator
import math

class GPS_class():
    def __init__(self):
        pass

    def GPSparser(self, data):
        gps_data = []

        idx_rmc = data.find('GNRMC')
        if data[idx_rmc:idx_rmc + 5] == "GNRMC":
            data = data[idx_rmc:]
            #print(data)
            if self.checksum(data):
                spliteddata = data.split(",")
                if spliteddata[2] == 'V':
                    print("data invalid")
                    return 0

                elif spliteddata[2] == 'A':
                    gps_data.append(float(spliteddata[1]))
                    if spliteddata[4] == 'N':
                        gps_data.append(float(spliteddata[3]))
                    else:
                        gps_data.append(-1.0 * float(spliteddata[3]))

                    if spliteddata[6] == 'E':
                        gps_data.append(float(spliteddata[5]))
                    else:
                        gps_data.append(-1.0 * float(spliteddata[5]))

                    if not spliteddata[7] == '':
                        gps_data.append(float(spliteddata[7]))
                    else:
                        gps_data.append(-1.0)
                    if not spliteddata[8] == '':
                        gps_data.append(float(spliteddata[8]))
                    else:
                        gps_data.append(-1.0)

                    return gps_data  # list
            else:
                print("checksum error")
                return 0
        return 0

    def checksum(self, sentence):
        sentence = sentence.strip('\n')
        nmeadata, cksum = sentence.split('*', 1)
        calc_cksum = reduce(operator.xor, (ord(s) for s in nmeadata), 0)
        #print(int(cksum, 16), calc_cksum)  # checksum

        if int(cksum, 16) == calc_cksum:
            return True
        else:
            return False

    '''def Kalmann_Filter(self, pre_gps_data, gps_data):
        if gps_data == 0:
            return
        if gps_data == None:
            return
        K = 0
        K_data = [0, 0, 0, 0, 0]
        K_data[0] = K * pre_gps_data[0] + (1 - K) * float(gps_data[0])
        K_data[1] = K * pre_gps_data[1] + (1 - K) * float(gps_data[1])
        K_data[2] = K * pre_gps_data[2] + (1 - K) * float(gps_data[2])
        K_data[3] = K * pre_gps_data[3] + (1 - K) * float(gps_data[3])
        K_data[4] = K * pre_gps_data[4] + (1 - K) * float(gps_data[4])
        
        return K_data'''

    def isis(self, data):
        isNone = False
        if data == None:
            isNone = True
        return isNone


class DMMtoDD():
    def __init__(self):
        pass

    def dmm_to_dd(self,res_data):
        #DD_res_data=[0,0]
        DD_res_data = float(res_data) // 100 + float(res_data) % 100 / 60 
        #DD_res_data[1] = float(res_data[2]) // 100 + float(res_data[2]) % 100 / 60
        #self.DD_res_data = [DD_res_data[0], DD_res_data[1]]
        return DD_res_data

class DDtoTM():
    def __init__(self):
        pass

    def Geo2Tm(self,plon, plat):    
        lon = plon
        lat = plat
        lon = lon * math.pi / 180
        lat = lat * math.pi / 180
        m_arScaleFactor = 1
        m_arLonCenter = 2.21661859489632
        m_arLatCenter = 0.663225115757845
        m_arFalseNorthing = 500000.0
        m_arFalseEasting = 200000.0

        m_arMajor = 6378137.0
        m_arMinor = 6356752.3142

        temp = m_arMinor / m_arMajor

        m_dSrcEs = 1.0 - temp * temp
        m_dDstEs = 1.0 - temp * temp
        m_dDstEsp = m_dDstEs / (1.0 - m_dDstEs)
        m_dDstE0 = 1.0 - 0.25 * m_dDstEs * (1.0 + m_dDstEs / 16.0 * (3.0 + 1.25 * m_dDstEs))
        m_dDstE1 = 0.375 * m_dDstEs * (1.0 + 0.25 * m_dDstEs * (1.0 + 0.46875 * m_dDstEs))
        m_dDstE2 = 0.05859375 * m_dDstEs * m_dDstEs * (1.0 + 0.75 * m_dDstEs)
        m_dDstE3 = m_dDstEs * m_dDstEs * m_dDstEs * (35.0 / 3072.0)
        m_dDstMl0 = m_arMajor * (m_dDstE0 * m_arLatCenter - m_dDstE1 * math.sin(2.0 * m_arLatCenter) + m_dDstE2 * math.sin(4.0 * m_arLatCenter) - m_dDstE3 * math.sin(6.0 * m_arLatCenter))

        m_dDstInd = 0.0

        delta_lon = lon - m_arLonCenter # Delta longitude (Given longitude - center longitude)
        sin_phi = math.sin(lat) # sin and cos value
        cos_phi = math.cos(lat)

        b = 0 # temporary values
        x = 0.5 * m_arMajor * m_arScaleFactor * math.log10((1.0 + b) / (1.0 - b))
        con = math.acos(cos_phi * math.cos(delta_lon) / math.sqrt(1.0 - b * b)) # cone constant, small m

        al = cos_phi * delta_lon # temporary values
        als = al * al
        c = m_dDstEsp * cos_phi * cos_phi
        tq = math.tan(lat)
        t = tq * tq
        con = 1.0 - m_dDstEs * sin_phi * sin_phi
        n = m_arMajor / math.sqrt(con)
        ml = m_arMajor * (m_dDstE0 * lat - m_dDstE1 * math.sin(2.0 * lat) + m_dDstE2 * math.sin(4.0 * lat) - m_dDstE3 * math.sin(6.0 * lat))

        x = m_arScaleFactor * n * al * (1.0 + als / 6.0 * (1.0 - t + c + als / 20.0 * (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * m_dDstEsp))) + m_arFalseEasting
        y = m_arScaleFactor * (ml - m_dDstMl0 + n * tq * (als * (0.5 + als / 24.0 * (5.0 - t + 9.0 * c + 4.0 * c * c + als / 30.0 * (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * m_dDstEsp))))) + m_arFalseNorthing
        plon = x
        plat = y
        
        return x,y





def main():
    pub = rospy.Publisher('/GPS_RealTime', GPS, queue_size=10)
    rospy.init_node('GPS', anonymous=False)
    rate = rospy.Rate(8)
    dd=DMMtoDD()
    tm=DDtoTM()
    msg = GPS()
    gps = GPS_class()

    ser = serial.Serial(port="/dev/ttyACM1", baudrate=38400, timeout=0.1)
    res_data = [0, 3744.79912, 12665.34976, 0, 0] #initial data

    while not rospy.is_shutdown():
        data = ser.readline()
        #rospy.loginfo(data)
        #res_data = [0, 3744.79912, 12665.34976, 0, 0]
        #pre_gps_data = res_data

        if gps.GPSparser(data) == 0:
            continue

        elif gps.isis(data) == False:
            #res_data = gps.Kalmann_Filter(pre_gps_data, gps.GPSparser(data))
            res_data = gps.GPSparser(data)
            print(float(res_data[1]), float(res_data[2]))
            #res_data = gps.GPSparser(data)
            #rospy.loginfo(res_data)

        

        msg.x, msg.y = tm.Geo2Tm(dd.dmm_to_dd(res_data[1]), dd.dmm_to_dd(res_data[2]))        
        msg.velocity = float(res_data[3])

        pub.publish(msg)

        #rospy.loginfo(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
