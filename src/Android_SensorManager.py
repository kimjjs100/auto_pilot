import math

# Source : https://android.googlesource.com/platform/frameworks/base/+/master/core/java/android/hardware/SensorManager.java#22

def getRotationMatrix(R, I, gravity, geomagnetic):
    Ax = gravity[0]
    Ay = gravity[1]
    Az = gravity[2]

    normsqA = (Ax * Ax + Ay * Ay + Az * Az)
    g = 9.81
    freeFallGravitySquared = 0.01 * g * g

    if (normsqA < freeFallGravitySquared):
        # gravity less than 10% of normal value
        print('fail normA')
        return False

    Ex = geomagnetic[0]
    Ey = geomagnetic[1]
    Ez = geomagnetic[2]
    Hx = Ey * Az - Ez * Ay
    Hy = Ez * Ax - Ex * Az
    Hz = Ex * Ay - Ey * Ax
    normH = math.sqrt(Hx * Hx + Hy * Hy + Hz * Hz)

    if normH < 0.1:
        # device is close to free fall (or in space?), or close to
        # magnetic north pole. Typical values are  > 100.
        print('fail nomrH')
        return False

    invH = 1.0 / normH
    Hx *= invH
    Hy *= invH
    Hz *= invH

    invA = 1.0 / math.sqrt(Ax * Ax + Ay * Ay + Az * Az)
    Ax *= invA
    Ay *= invA
    Az *= invA

    Mx = Ay * Hz - Az * Hy
    My = Az * Hx - Ax * Hz
    Mz = Ax * Hy - Ay * Hx
    if (R != None) :
        if len(R) == 9:
            R[0] = Hx;     R[1] = Hy;     R[2] = Hz
            R[3] = Mx;     R[4] = My;     R[5] = Mz
            R[6] = Ax;     R[7] = Ay;     R[8] = Az
        elif len(R) == 16:
            R[0]  = Hx;    R[1]  = Hy;    R[2]  = Hz;   R[3]  = 0
            R[4]  = Mx;    R[5]  = My;    R[6]  = Mz;   R[7]  = 0
            R[8]  = Ax;    R[9]  = Ay;    R[10] = Az;   R[11] = 0
            R[12] = 0;     R[13] = 0;     R[14] = 0;    R[15] = 1
        
    if (I != None):
        # compute the inclination matrix by projecting the geomagnetic
        # vector onto the Z (gravity) and X (horizontal component
        # of geomagnetic vector) axes.
        invE = 1.0 / math.sqrt(Ex * Ex + Ey * Ey + Ez * Ez)
        c = (Ex * Mx + Ey * My + Ez * Mz) * invE
        s = (Ex * Ax + Ey * Ay + Ez * Az) * invE

        if (len(I) == 9):
            I[0] = 1;     I[1] = 0;     I[2] = 0
            I[3] = 0;     I[4] = c;     I[5] = s
            I[6] = 0;     I[7] = -s;     I[8] = c
        elif (len(I) == 16):
            I[0] = 1;     I[1] = 0;     I[2] = 0
            I[4] = 0;     I[5] = c;     I[6] = s
            I[8] = 0;     I[9] = -s;     I[10] = c
            I[3] = I[7] = I[11] = I[12] = I[13] = I[14] = 0
            I[15] = 1

    return True
    
def getOrientation(R, values):
    '''
    4x4 (length=16) case:
        R[ 0]   R[ 1]   R[ 2]   0  
        R[ 4]   R[ 5]   R[ 6]   0 
        R[ 8]   R[ 9]   R[10]   0 
           0       0       0    1  
    
    3x3 (length=9) case:
        R[ 0]   R[ 1]   R[ 2]  
        R[ 3]   R[ 4]   R[ 5]  
        R[ 6]   R[ 7]   R[ 8]  
    '''
    if (len(R) == 9):
        values[0] = math.atan2(R[1], R[4])
        values[1] = math.asin(-R[7])
        values[2] = math.atan2(-R[6], R[8])
    else:
        values[0] = math.atan2(R[1], R[5])
        values[1] = math.asin(-R[9])
        values[2] = math.atan2(-R[8], R[10])
    
    return values

def calculation():
    gravity = [0, 0, -9.8]
    geomagnetic = [0.000015 *  10**6, 0.000025 *  10**6, 0.000019 *  10**6] 
    print(geomagnetic)
    R = [0]*9
    orientation = [0] * 3
    success = getRotationMatrix(R, None, gravity, geomagnetic)
    if success is True:
        getOrientation(R, orientation)
        mAzimuth = orientation[0] * 180 / math.pi
        mPitch = orientation[1] * 180 / math.pi
        mRoll = orientation[2] * 180 / math.pi

        print(mAzimuth)
    
def main():
    pass



if __name__ == "__main__":
    main()