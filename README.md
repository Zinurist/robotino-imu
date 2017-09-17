

### Connection to the ROS network
In order to publish/subscribe to topics on the Raspberry Pi/Robotino ROS system, some variables in the environment need to be set. These point to the ROS network's location. They are set in ros_network.sh.
Both systems might need each other registered as hosts.

### Collecting data
Data is collected using recorder.py, that runs on the Raspberry Pi. The ip of the persistance system needs to be set in recorder.py. The script assumes, that the platform-model of the persistance system doesn't require all parameters, same for the media-model.

For the imu RTIMULib2 needs to be installed on the Raspberry Pi, and the RTIMULib.ini file needs to be present.

### Control the robot
The script controller2.py provides a simple control interface for the robot for testing. The controls are summed up in the script.
The same controls are used to control the robot for the live demo, see the Keras script in the ml folder.
See speeds.txt for a rough measurement of the Robotino's maximum velocity.

### Testing
Test files (all files that start with "test_") are not automated tests. They are used manually to see if all systems work as expected.
