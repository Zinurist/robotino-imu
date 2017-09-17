#!/usr/bin/env python


"""
This script provides an easy interface for the imu. Calling get_vals will return all imu data in the buffer.
For an alternative version see imu_o.py. Contrary to imu_o.py, this script polls the imu in a seperate thread, and stores the polled data in its own buffer.
"""

import sys
sys.path.append('.')
import RTIMU
import os.path
import time
from threading import Thread,Lock

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


#a separate thread is used to poll the imu
val = []
run = True
mutex = Lock()

def get_val_t():
    global val
    while run:
        #at rec poll intervall:
        time.sleep(poll_interval/1000.0)
        mutex.acquire()
        while imu.IMURead():
            val.append(imu.getIMUData())
        mutex.release()

#create and start thread
thr = Thread(target=get_val_t)
thr.daemon = True
thr.start()

"""
Returns the imu data, just like in imu_o.py.
"""
def get_vals():
    mutex.acquire()
    global val
    vals = val
    val = []
    mutex.release()
    return vals


"""
Stops the thread.
"""
def stop():
    global run
    run = False


