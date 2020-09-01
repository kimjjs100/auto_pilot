import csv
import math
import numpy
import pandas


class DDtoTM():
    def __init__(self):
        pass

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
        
        return [x, y]



class DMStoDD():
    def __init__(self):
        pass

    def dms_to_dd(self, res_data):
        DD_res_data = []
        DD_res_data[0] = float(res_data[0]) // 100 + float(res_data[0]) % 100 / 60
        DD_res_data[1] = float(res_data[1]) // 100 + float(res_data[1]) % 100 / 60
        self.DD_res_data = [DD_res_data[0], DD_res_data[1]]
        return DD_res_data


def csv2list(filename):
    file = open(filename, 'r')
    csvfile = csv.reader(file)
    lists = []
    for item in csvfile:
        lists.append(item)
    return lists




tm = DDtoTM()
dd = DMStoDD()
tm_csv = []
dd_csv = []
x=[]
y=[]
a=[]

data=pandas.read_csv('original.csv')
for i in data:
    dd_csv.append(numpy.float64(data[i]//100+numpy.float64(data[i])%100/60))

for i in range(len(dd_csv[0])):
    x.append(dd_csv[0][i])
    y.append(dd_csv[1][i])

for i in range(len(x)):
    a.append(tm.Geo2Tm(x[i],y[i]))

print
dataframe=pandas.DataFrame(a)
dataframe.to_csv("TM_Converted.csv",header=False,index=False)




