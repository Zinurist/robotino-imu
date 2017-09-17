#!/usr/bin/env python

"""
Functions used to see whether movement returns the data in the expected format.
"""

#ROS
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import movement as mv
import movement_types as mt


sample_run = 0
#unique id for easy filtering
sample_id = 1667


def save_sample(token, run, samples, labels, hertz, vel, ang):
    print("saving sample...")
    img = samples['image']
    print(type(img))
    print(len(img))
    print(type(img[0]))
    print(img[0].shape)
	


def data_collector(hertz=10, samples=10):
    token=0

    #init publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('collector', anonymous=True)
    mvs = mv.MovementSet(pub, hertz)

    #repeat movement for each sample
    for k in range(samples):
        if rospy.is_shutdown():
            return

        labels,fn = mt.get_mv(mvs, 'curve_s')
        samples,vel,ang = fn()
            
        #stop robotino
        pub.publish(Twist())
        print("sample no. %s done" % k)
        save_sample(token, sample_run+k, samples, labels, hertz, vel, ang)
	


if __name__ == '__main__':
    try:
        data_collector(samples=5)
    except rospy.ROSInterruptException:
        pass


