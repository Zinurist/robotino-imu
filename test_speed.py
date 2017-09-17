#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import math
import pygame

"""
Test function used to measure linear and angular speed of the robotino.
The speed and seconds set in this function can be compared with the real measured data, to get an idea of how fast the robot is.

@param linear: if true, the linear speed is tested, else angular
"""
def controller(linear=True):     
	
    hertz = 10
    secs = 2
   
    #init publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(hertz)

    vel = 0.09
    ang = math.pi*0.3*0.5

    vel2 = 0.0
    ang2 = 0.0

    #movement data
    vec_zero = Vector3(0,0,0)
    vec_fwd = Vector3(vel,0,0)
    vec_bwd = Vector3(vel2,0,0)
    vec_left = Vector3(0,0,ang)
    vec_right = Vector3(0,0,-ang2)

    for i in range(int(hertz*secs)):    
        if linear: pub.publish(Twist(vec_fwd, vec_zero))
        else: pub.publish(Twist(vec_fwd, vec_left))
        rate.sleep()
      
    pub.publish(Twist(vec_zero, vec_zero))
    rate.sleep()
        
    for i in range(int(hertz*secs)):      
        if linear: pub.publish(Twist(vec_bwd, vec_zero))
        else: pub.publish(Twist(vec_zero, vec_right))
        rate.sleep()

    pub.publish(Twist(vec_zero, vec_zero))


if __name__ == '__main__':
    controller()
