These script were used to create videos of the live demo, and are irrelevant to the project. For those interested, here is a short explanation of how the videos were made:

A Video of the robot was filmed whilst also recording the output of the neural network and annotating that data with timestamps.

video_recorder.py records a second video from a webcam and renders the video with the current timestamp. The recorded video can be compared with the first video to synchronize the time of the first video with the timestamps.

video_converter.py then uses the recorded and timestamped network output, the recorded video and the synchronize information to render a new video of the robot together with the networks output.