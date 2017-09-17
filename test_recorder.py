#!/usr/bin/env python

"""
Small testing class for the recorder. Can be used to test the connection to the database (whether connecting is successful and data is sent and stored in the mongoDB).
"""


#database
import json
import requests

#url and port of database
base_url = 'https://192.168.43.208'
port = 8081
#sample data
experiment_id = u'5925a5f3de3f745139c47af3'
platform_id = u'5925af9f6bc97d57deb8fdcc'
sample_run = -1
#unique id for easy filtering
sample_id = -1

#connect to persistence system
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


def save_sample(token, run, samples, labels, hertz, vel, ang):
    print("saving sample...")
    samplejson = []
    annotjson = []
    samplejson.append({'tags' : ['imu','accel'], 'data' : samples['accel']})
    samplejson.append({'tags' : ['imu','gyro'], 'data' : samples['gyro']})
    samplejson.append({'tags' : ['imu','compass'], 'data' : samples['compass']})
    annotjson.append({'domains' : ['movement'], 'linear' : {'x':vel,'y':0,'z':0}, 'angular' : {'x':0,'y':0,'z':ang}})
    annotjson.append({'domains' : labels})
    annotjson.append({'domains' : ['id'], 'value' : sample_id})
    annotjson.append({'domains' : ['hertz'], 'value' : hertz})
    
    sample_url = base_url + ':' + str(port) + '/api/v1/samples/'
    sample_headers = {'Authorization': token}
    sample_payload = {'experiment': experiment_id, 'run': run, 'platform': platform_id, 'media' : samplejson, 'annotations' : annotjson }
    sample_response = requests.post(sample_url, json=sample_payload, headers=sample_headers, verify=False).json()

    print(sample_response)
    if 'error' in sample_response:
        print("error when saving sample")
    else:
        print("saved!")
	


def data_collector(hertz=10, samples=10):
    token = connect()

    #repeat movement for each sample
    labels = ["curve_m"]
    for k in range(samples):
        samples = {'accel':[[1,2,3],[11,22,33]], 'gyro':[[4,5,6],[44,55,66]], 'compass': [[7,8,9],[77,88,99]]}
        vel = 1
        ang = 1.5
        print("sample no. %s done" % k)
        save_sample(token, sample_run+k, samples, labels, hertz, vel, ang)
	


if __name__ == '__main__':
    data_collector(samples=10)


