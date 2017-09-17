#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import imu_o as im
#import random

"""
Start a ROS publisher that sends the imu data using the Twist message object.
@param hertz: rate at which to collect imu data
@param secs: how long to send, only used for testing, source code needs to be modified for this parameter to have an effect
"""
def imu_publisher(hertz=80, secs=5):
    pub = rospy.Publisher('/odom', Twist, queue_size=10)
    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(hertz)

    vals = []
    while not rospy.is_shutdown():
    #for i in range(secs*hertz):
        vals = im.get_vals()
        
        for v in vals:
            v1 = Vector3(v['accel'][0],v['accel'][1],v['accel'][2])
            v2 = Vector3(v['gyro'][0],v['gyro'][1],v['gyro'][2])
            #v1 = Vector3(random.randint(-10,10),random.randint(-10,10),random.randint(-10,10))
            #v2 = Vector3(random.randint(-10,10),random.randint(-10,10),random.randint(-10,10))
            t = Twist(v1,v2)
            pub.publish(t)
        rate.sleep()
    im.stop()

if __name__ == '__main__':
    imu_publisher()
