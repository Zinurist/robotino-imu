#!/usr/bin/env python

"""
This script provides an easy interface for the imu. Calling get_vals will return all imu data in the buffer.
For an alternative version see imu_t.py. This version however was used for recording.
"""

import sys
sys.path.append('.')
import RTIMU
import os.path

SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
    print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded");

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

"""
Returns all imu data from the buffer as a list, using RTIMULib2. A single element of this list contains imu data from one timestep as a dict. The most important keys in this dict are "accel", "gyro" and "compass" for the 3-axis vectors of the respective sensors.
@return list of the imu data
"""
def get_vals():
    data = []
    while imu.IMURead():
        data.append(imu.getIMUData())
    return data
"""
Only defined to make changing from imu_t to imu_o easier.
"""
def stop():
    return 0

