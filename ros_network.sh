#!/bin/bash
#need to be set so that the ROS network of the robotino+pi is used
export ROS_HOSTNAME=zinunb
export ROS_IP=$(hostname -I)
export ROS_MASTER_URI=http://pi:11311
