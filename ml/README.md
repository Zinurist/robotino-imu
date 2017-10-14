In this folder all files for the machine learning part are stored.

## Labelling data
The "trunc label.ipynb" Jupyter Notebook is used to label the data (automatically and by hand) and then save the data in the JSON format. 

## Neural Network Models
The network models are defined in RobotinoKeras.ipynb. Training, testing and the live demo are all started in this Notebook. RobotinoKerasSimple.ipynb is a simplified version of this file without all the different model definitions.

These scripts save the network models in tmp/keras/ (relative to their location), these folder might need to be created manually beforehand. When loading sample data from a JSON file (the default for the first two tasks, the third task loads the data directly from a running database), the file should be in the same folder as the notebook.

The model_type parameter defines the classification task, the model_num parameter. The model types are:
- 0: curves
- 1/2: obstacles without/with foot
- 6: curves sm/ml
- 10/11/12: angles in 45/30/15 degree steps


The networks used for the thesis are (in the format type_num)
- 0/1/2_9 (RNNs for first two tasks)
- 10/11_34 (RNNs for angles task
- 12_36 (same model as 34, but trained with different optimizer)
- 11_39/45 (DNNs for angles task)


## Working with the Notebook
The notebook contains different parameters at the start that control the flow of the script. Here is a description of these parameters:
- load_test: Load the test data instead of the training/validation data. Used after training to evaluate the model. The test data is loaded as validation data in the samples object.
- predict_mode: Set this to true when using the network for prediction rather than training. In this mode the architecture of the network is slightly different compared to training: the model is set to be stateful, meaning the state of a recurrent cell is saved after feeding it data. In this mode the sample data also isn't unrolled (for further evaluation of the model).
- live_demo: Set to true to start the live demo (predict mode is needed). It might be more convenient to have this be false and enable the live demo manually later on.
- train_network: If true, the training function will be called. This parameter is used to more easily disable training when doing a quick evaluation of the network.
- load_rnn/load_only_weights: Whether or not to load a saved model, or create a new one. When the model doesn't exist yet, this needs to be false. 


## Other files
The other files in this folder are explained here:
- samples.py: Script that load and manages the trainings and test data.
- model_types.py: Gives necessary information for loading data for the tasks.
- imu_listener.py: Starts a controller for the robot and subscribes to the topic of the imu_publisher.py, feeds the received imu data through the network and displays the output of the network. Effectively the live demo of this project.
- imu_graph.py: Subscribes to the topic of the imu_publisher.py and draws a graph of the imu data (used for testing).
- ExampleTF(2).ipynb: Example code used to generate Tensorboard graphs for the thesis.
