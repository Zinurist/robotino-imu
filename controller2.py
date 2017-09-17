#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import math
import pygame

"""
Starts a controller with which the Robotino can be controlled. It sends control data via ROS to the Raspberry Pi, where the Robotino node is running.
It assumes that a pygame instance is set up and running. Controlling the robot is done via pygame:
q - quit
w/s - drive forward/backwards
a/d - turn left/right at the speed of curve_m
hold shift - affects turn speed, now at the speed of curve_s
hold ctrl - affects turn speed, now at the speed of curve_l
"""
def controller():
        
    #init publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10)
    
    vel = 0.09
    ang = math.pi*0.3
    
    #movement data
    vec_zero = Vector3(0,0,0)
    vec_fwd = Vector3(vel,0,0)
    vec_bwd = Vector3(-vel,0,0)
    
    #angular and linear movement
    mv_l = [vec_zero]
    mv_a = [0]

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pub.publish(Twist())              
                return
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w: mv_l.insert(0,vec_fwd)
                elif event.key == pygame.K_s: mv_l.insert(0,vec_bwd)
                elif event.key == pygame.K_a: mv_a.insert(0,ang)
                elif event.key == pygame.K_d: mv_a.insert(0,-ang)
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_w: mv_l.remove(vec_fwd)
                elif event.key == pygame.K_s: mv_l.remove(vec_bwd)
                elif event.key == pygame.K_a: mv_a.remove(ang)
                elif event.key == pygame.K_d: mv_a.remove(-ang)
        
        mods = pygame.key.get_mods()
        if mods & pygame.KMOD_SHIFT:
            vec_curve = Vector3(0,0,mv_a[0]*0.7)
        elif mods & pygame.KMOD_CTRL:
            vec_curve = Vector3(0,0,mv_a[0]*0.2)
        else:
            vec_curve = Vector3(0,0,mv_a[0]*0.4)
           
        pub.publish(Twist(mv_l[0], vec_curve))
        rate.sleep()
        



if __name__ == '__main__':
    pygame.init()
    pygame.display.set_mode((300,300))
    controller()
    pygame.quit()




