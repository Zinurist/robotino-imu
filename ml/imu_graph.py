#!/usr/bin/env python

"""
Small script that plots the imu data live, the imu publisher script needs to be running on the Raspberry Pi.
"""
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#data
plots = 6
size = 400
x = range(size)
y = []
for i in range(plots):
    y.append([0]*size)

def process_imu(val):
    global y
    data = []
    data.append(val.linear.x)
    data.append(val.linear.y)
    data.append(val.linear.z)
    data.append(val.angular.x)
    data.append(val.angular.y)
    data.append(val.angular.z)
    for i in range(plots):
        y[i].append(data[i])
        del y[i][0]


#ROS subscriber
sub = rospy.Subscriber('/odom', Twist, process_imu)
rospy.init_node('graph', anonymous=True)


#Plot data
fig = plt.figure()
ax = [None]*plots
for i in range(plots):
    ax[i] = fig.add_subplot(plots,1,i+1)

def animate(i):
    for i in range(plots):
        ax[i].clear()
        ax[i].plot(x,y[i])

ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()

sub.unregister()














