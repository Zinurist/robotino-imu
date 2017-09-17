#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import math


"""
A small test to see whether the robotino uses the linear vector as a direction.
"""
def controller(deg = 60):	  

    hertz = 80   	
    vel = 0.09
    angle = math.pi*2*deg/360
    secs = 4

    velY = math.sin(angle)*vel
    velX = math.cos(angle)*vel

    vec_zero = Vector3(0,0,0)
    vec_angle = Vector3(velX,velY,0)	

    #init publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(hertz)

    for i in range(secs*hertz):
        pub.publish(Twist(vec_angle, vec_zero))
        rate.sleep()

    pub.publish(Twist())


if __name__ == '__main__':
    controller(330)
    controller(360 - 150)
    controller(90)
