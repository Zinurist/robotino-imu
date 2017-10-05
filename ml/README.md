In this folder all files for the machine learning part are stored.

## Labelling data
The "trunc label.ipynb" Jupyter Notebook is used to label the data (automatically and by hand) and then save the data in the JSON format. 

## Neural Network Models
The network models are defined in RobotinoKeras.ipynb. Training, testing and the live demo are all started in this Notebook. RobotinoKerasSimple.ipynb is a simplified version of this file without all the different model definitions.

These scripts save the network models in tmp/keras/ (relative to their location), these folder might need to be created manually beforehand.

## Other files
The other files in this folder are explained here:
- samples.py: Script that load and manages the trainings and test data.
- model_types.py: Gives necessary information for loading data for the tasks.
- imu_listener.py: Starts a controller for the robot and subscribes to the topic of the imu_publisher.py, feeds the received imu data through the network and displays the output of the network. Effectively the live demo of this project.
- imu_graph.py: Subscribes to the topic of the imu_publisher.py and draws a graph of the imu data (used for testing).
- ExampleTF(2).ipynb: Example code used to generate Tensorboard graphs for the thesis.
