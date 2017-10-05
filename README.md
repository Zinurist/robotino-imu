
## System description
The Robotino is connected to the Raspberry Pi (via ethernet). On the Pi runs the ROS system with the Robotino Node started to control the robot.
The recorded data is processed and stored on a seperate host.
The host and the Pi are connected via an extra network, preferably a private Wi-Fi network with static ip addresses. 

## Installation and Setup
The following needs to be setup in order to use the scripts and record data:

The Raspberry Pi connected to the Robotino needs the RTIMULib2 installed and working (for Python 2). To use the library, the RTIMULib.ini file needs to be in the same folder as the recorder script.
On the Pi ROS and the robotino_node need to be running. ROS can be started by simply calling `roscore`. The robotino_node is started after connecting the Robotino and then calling:
```
roslaunch robotino_node robotino_node.launch hostname:=<hostname>
```
where hostname is the ip address of the Robotino in the ethernet network.

The host needs the mongo database and the persistance system running and connected. The recording script assumes that the platform and experiment are already registered in the persistance system. If not, it tries to create them. In this case, slight changes need to be made to the models in the persistance system, as the script doesn't supply all needed data: 
- In the media model (in sample.js) configuration required is set to false
- In the platform model (in platform.js) version required is set to false

Alternatively the recorder script can be modified to include this data.


## Connection to the ROS network
The host needs to be connected to the Raspberry Pi through a network. In order to publish/subscribe to topics on the Pi ROS system, some variables in the environment of the host need to be set. These point to the ROS network's location. They are set in ros_network.sh:
- ROS_HOSTNAME/IP: name and ip address of the host system, this name also needs to be registered as a host on the Pi
- ROS_MASTER_URI: uri of the Pi, in this case its ip address and the port on which ROS is listening
Both systems need each other registered as hosts (register each others ip address and hostname in /etc/hosts). Make sure the host system doesn't block ROS with a firewall.

## Collecting data
Data is collected using recorder.py, that runs on the Raspberry Pi. At the top of the script are various parameters that need to be set in order to use it:
- base_url and port: The url (or ip address) of the host where the persistance system is running and the port on which it is listening.
- experiment_id and platform_id: The ids that the persistance system assigns to the experiment and platform object for this recording. When left empty, the script tries to create new ones (note that the persistance system needs to be modified as not all parameters for these models are supplied by the script, see setup above).
- sample_run and sample_id: sample_run is an index incremented after each run, and sample_id is used for the whole recording session. These can be used for easier filtering later on.

The script per default collects 20 samples (this can be changed at the bottom of the script). The usage parameters for the script are:
```
./recorder.py <mv_type> <left> <sample_id> <degree>
```

The parameter degree (an integer as degrees between 0 and 180) is only relevant when mv_type is angle, and left is either left for left curves/angles or anything else for right curves/angles. The allowed values for mv_type are detailed in movement_types.py.


## Control the robot
The script controller2.py provides a simple control interface for the robot for testing. The controls are summed up in the script.
The same controls are used to control the robot for the live demo, see the Keras script in the ml folder.

The script imu_publisher.py can be started on the Pi to send imu data to the ROS network. The data is published to the topic "/odom" and consists of 3-dim linear and angular vectors for the accelerometer and gyroscope data.

See speeds.txt for a rough measurement of the Robotino's maximum velocity.

## Testing
Test files (all files that start with "test_") are not automated tests. They are used manually to verify that all systems work as expected.
