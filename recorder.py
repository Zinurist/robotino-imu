#!/usr/bin/env python

"""
The recorder script, that moves the robot and records imu data, and then sends it to the persistance system. This script is run on the Raspberry Pi, which needs RTIMULib2 installed and the imu connected to it. On the Raspberry Pi, the robotino node needs to be running to control the robot.
"""

#ROS
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import movement as mv
import movement_types as mt
import math
#import imu_t as imu

#database
import json
import requests

#url and port of database
base_url = 'https://192.168.43.208'
port = 8081
#sample data
experiment_id = u'5925a5f3de3f745139c47af3'
platform_id = u'5925af9f6bc97d57deb8fdcc'
sample_run = 0
#unique id for easier filtering
sample_id = 1667

"""
Connect to the persistance system and returns the authentication token.
@return the authentication token
"""
def connect():
    #login
    print("connecting to database at %s:%s" % (base_url, port))
    login_url = base_url + ':' + str(port) + '/v1/authenticate/'
    login_payload = {'username': 'zinu', 'password': 'testo123'}
    login_response = requests.post(login_url, login_payload, verify=False).json()
    token = login_response['token']
    author = login_response['user']['_id']

    #create experiment if not yet done
    global experiment_id
    if experiment_id == '':
        experiment_url = base_url + ':' + str(port) + '/api/v1/experiments/'
        experiment_headers = {'Authorization': token}
        experiment_payload = {'name': 'robotino curve', 'author': author, 'description': 'Robotino curve experiment'}
        experiment_response = requests.post(experiment_url, experiment_payload, 	headers=experiment_headers, verify=False).json()
        experiment_id = experiment_response['_id']
        print("experiment ID is %s, save for later use" % experiment_id)

    global platform_id
    if platform_id == '':
        platform_url = base_url + ':' + str(port) + '/api/v1/platforms/'
        platform_headers = {'Authorization': token}
        platform_payload = {'name': 'Robotino'}
        platform_response = requests.post(platform_url, platform_payload, 	headers=platform_headers, verify=False).json()
        print(platform_response)
        platform_id = platform_response['_id']
        print("platform ID is %s, save for later use" % platform_id)
            
    print("connected!")
    return token

"""
Sends the provided sample data to the persistance system.

@param token: the authentication token from the connect-call
@param run: the run of this sample
@param samples: the sample data (see movement_types for the format)
@param labels: the labels for the sample (e.g. curve_m_r and curve_m)
@param hertz: rate used to control the robotino in hertz
@param vel: linear velocity in x direction (forward) when controlling the robot during the sample
@param ang: angular velocity in z direction
@param radius: the radius in case of a curve (estimated)
"""
def save_sample(token, run, samples, labels, hertz, vel, ang, radius=0):
    print("saving sample...")
    samplejson = []
    annotjson = []
    samplejson.append({'tags' : ['imu','accel'], 'data' : samples['accel']})
    samplejson.append({'tags' : ['imu','gyro'], 'data' : samples['gyro']})
    samplejson.append({'tags' : ['imu','compass'], 'data' : samples['compass']})
    samplejson.append({'tags' : ['cam','image'], 'data' : samples['image']})
    annotjson.append({'domains' : ['movement'], 'linear' : {'x':vel,'y':0,'z':0}, 'angular' : {'x':0,'y':0,'z':ang}, 'radius' : radius})
    annotjson.append({'domains' : labels})
    annotjson.append({'domains' : ['id'], 'value' : sample_id})
    annotjson.append({'domains' : ['run'], 'value' : run})
    annotjson.append({'domains' : ['hertz'], 'value' : hertz})

    sample_url = base_url + ':' + str(port) + '/api/v1/samples/'
    sample_headers = {'Authorization': token}
    sample_payload = {'experiment': experiment_id, 'run': run, 'platform': platform_id, 'media' : samplejson, 'annotations' : annotjson }
    sample_response = requests.post(sample_url, json=sample_payload, headers=sample_headers, verify=False).json()

    #print(sample_response)
    if 'error' in sample_response:
        print("error when saving sample")
    else:
        print("saved!")
	

"""
The collector that collects a given number of samples for a given type of sample. It calls the connect function and uses the save_sample function to send the sample data.
@param hertz: rate for controlling the robot
@param samples: number of samples to collect
@param mv_type: the type of sample to collect (see movement_types)
@param left: if left or right (relevant for curves and angles)
@param degree: what degree to use for the angle movement
"""
def data_collector(hertz=50, samples=20, mv_type='curve_m', left=False, degree=None):
    token = connect()

    #init publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('collector', anonymous=True)
    mvs = mv.MovementSet(pub, hertz)

    print('%s - %s' % (mv_type, 'left' if left else 'right'))

    #repeat movement for each sample
    for k in range(samples):
        if rospy.is_shutdown():
                return
                
        #type of movement
        #labels = ["curve_m"]
        #samples,vel,ang = mvs.mv_curve_middle()
        labels,fn = mt.get_mv(mvs, mv_type, left=left, degree=degree)
        samples,vel,ang,rad = fn()
                
        #stop robotino
        pub.publish(Twist())
        print("sample no. %s done" % k)
        save_sample(token, sample_run+k, samples, labels, hertz, vel, ang, rad)
	


if __name__ == '__main__':
    import sys
    usage = 'Usage: recorder.py <mv_type> <left> <sample_id> <degree>\n degree only relevant when mv_type is angle'
    if len(sys.argv) > 1 and (sys.argv[1] == '-help' or sys.argv[1] == '-h'):
        print(usage)
        sys.exit(0)
    
    mv_type = 'curve_m'
    left = False
    degree = None
    if len(sys.argv) > 1:
        mv_type = sys.argv[1]
    if len(sys.argv) > 2:
        left = sys.argv[2] == 'left'
    if len(sys.argv) > 3:
        sample_id = int(sys.argv[3])
    if len(sys.argv) > 4 and mv_type == 'angle':
        degree = int(sys.argv[4])
    try:
        data_collector(hertz=50, samples=20, mv_type=mv_type, left=left, degree=degree)
        #imu.stop()
    except rospy.ROSInterruptException:
        pass


