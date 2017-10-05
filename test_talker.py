#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import math


"""
A small test for controlling the robotino. (to test if the robotino moves and the ROS connections works)
"""
#angle, velocity and radius have same/similar unit (ex. rad/sec, meter/sec, meter)
def data_collector(velocity, radius, angle, hertz=20, secs=1, samples=50):
	if angle == None:
		angle = velocity/radius
	elif radius == None:
		#not really needed
		radius = velocity/angle
	else:
		print("Either angle or radius can be set, not both!")
		return

	#init publisher
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	rospy.init_node('trainer', anonymous=True)
	rate = rospy.Rate(hertz)

	#movement data
	vec_zero = Vector3(0,0,0)
	vec_fwd = Vector3(velocity,0,0)
	vec_curve = Vector3(0,0,angle)
	move_fwd = Twist(vec_fwd, vec_zero)
	move_curve = Twist(vec_fwd, vec_curve)
	
	#movement: first half straight, curve, second half straight
	first_half = hertz*secs/2
	sec_half = hertz*secs - first_half
	
	#repeat movement for each sample
	for k in range(samples):
		if rospy.is_shutdown():
			return
		
		for i in range(first_half):
			if i==1:
				#todo: start recording
				print("rec")
			pub.publish(move_fwd)
			rate.sleep()

		for i in range(hertz*secs): 
			pub.publish(move_curve)
			rate.sleep()

		for i in range(sec_half):
			if i==(sec_half-2):
				#todo: stop recording
				print("rec end")
			pub.publish(move_fwd)
			rate.sleep()

	#stob robotino at the end
	pub.publish(Twist(vec_zero, vec_zero))


if __name__ == '__main__':
    try:
        data_collector(velocity=1, radius=None, angle=math.pi/2, hertz=50, samples=60)
    except rospy.ROSInterruptException:
        pass
