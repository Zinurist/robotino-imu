#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import math

"""
Test function used to get (by measuring) the radius of the curves (angle speed needs to be set accordingly).
"""
def controller():	  
	
    hertz = 80   	
    vel = 0.09
    angle = math.pi*0.3*0.4
    extra = 45
    secs = math.pi/angle
    
    vec_zero = Vector3(0,0,0)
    vec_curve = Vector3(0,0,angle)	
    vec_fwd = Vector3(vel,0,0)	

    #init publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(hertz)

    for i in range(int(hertz*secs)+extra):
        pub.publish(Twist(vec_fwd, vec_curve))
        rate.sleep()
    pub.publish(Twist())


if __name__ == '__main__':
    controller()
