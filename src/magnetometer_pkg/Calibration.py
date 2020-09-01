#!/usr/bin/env python
import time
import threading
from lis3mdl import LIS3MDL


SAMPLE_SIZE = 2000
 
class KeyListener:
    """Object for listening for input in a separate thread"""
 
    def __init__(self):
        self._input_key = None
        self._listener_thread = None
 
    def _key_listener(self):
        while True:
            self._input_key = input()
 
    def start(self):
        """Start Listening"""
        if self._listener_thread is None:
            self._listener_thread = threading.Thread(
                target=self._key_listener, daemon=True
            )
        if not self._listener_thread.is_alive():
            self._listener_thread.start()
 
    def stop(self):
        """Stop Listening"""
        if self._listener_thread is not None and self._listener_thread.is_alive():
            self._listener_thread.join()
 
    @property
    def pressed(self):
        "Return whether enter was pressed since last checked" ""
        result = False
        if self._input_key is not None:
            self._input_key = None
            result = True
        return result

def main():    
    magnetometer = LIS3MDL()
    magnetometer.enableLIS()
 
    key_listener = KeyListener()
    key_listener.start()
 
    ############################
    # Magnetometer Calibration #
    ############################
 
    print("Magnetometer Calibration")
    print("Start moving the board in all directions")
    print("When the magnetic Hard Offset values stop")
    print("changing, press ENTER to go to the next step")
    print("Press ENTER to continue...")
    while not key_listener.pressed:
        pass

    min_x_Raw =[ ]
    min_y_Raw =[ ]
    min_z_Raw =[ ]

    max_x_Raw =[ ]
    max_y_Raw =[ ]
    max_z_Raw =[ ]
 
    mag_x, mag_y, mag_z = magnetometer.getMagnetometerRaw()
    min_x = max_x = mag_x
    min_y = max_y = mag_y
    min_z = max_z = mag_z
 
    for _ in range(SAMPLE_SIZE):
        mag_x, mag_y, mag_z = magnetometer.getMagnetometerRaw()
 
        print(
            "Magnetometer: X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
                mag_x, mag_y, mag_z
            )
        )
 
        min_x = min(min_x, mag_x)
        min_y = min(min_y, mag_y)
        min_z = min(min_z, mag_z)
 
        max_x = max(max_x, mag_x)
        max_y = max(max_y, mag_y)
        max_z = max(max_z, mag_z)


        min_x_Raw.append(min_x)
        min_y_Raw.append(min_y)
        min_z_Raw.append(min_z)

        max_x_Raw.append(max_x)
        max_y_Raw.append(max_y)
        max_z_Raw.append(max_z)
 
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

        #plt.title("XY Plane")
        #plt.plot(mag_x, mag_y, c="b",marker="o")

    x_scale = avg_rad / field_x
    y_scale = avg_rad / field_y
    z_scale = avg_rad / field_z
        
    #plt.show() 
    mag_calibration = (offset_x, offset_y, offset_z)
    print(
        "Final Magnetometer Calibration: X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
            offset_x, offset_y, offset_z
        )
    )

    print("")
    print("------------------------------------------------------------------------")
    print("Final Magnetometer Calibration Values: ", mag_calibration)

    print(
        "Scale Factor: X: {0:8.2f}, Y:{1:8.2f}, Z:{2:8.2f} uT".format(
            x_scale, y_scale, z_scale
        )
    )

    min_x_calib = [x - offset_x for x in min_x_Raw]
    min_y_calib = [y - offset_y for y in min_y_Raw]

    max_x_calib = [x - offset_x for x in max_x_Raw]
    max_y_calib = [y - offset_y for y in max_y_Raw]


if __name__ == "__main__":
    main()


