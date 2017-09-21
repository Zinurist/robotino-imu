import cv2
import time
from subprocess import call
import numpy as np
from threading import Thread

cap = cv2.VideoCapture("recording.MTS")
w,h = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
fps = int(cap.get(cv2.CAP_PROP_FPS))
print(fps)
fourcc = cv2.VideoWriter_fourcc(*'MJPG')

filename = 'recording_final'
out = cv2.VideoWriter('%s.avi' % filename, fourcc, fps, (w,h), isColor=True)


#load labels
f = open("history.dat", "r")
history = []
timedict = {}
for line in f:
    hist = line.split(' ')
    history.append((hist[0], float(hist[1])))
    as_int = int(float(hist[1]))
    if as_int not in timedict: timedict[as_int] = len(history)-1
f.close()

def get_index_of(timestamp):
    as_int = int(timestamp)
    index = timedict[as_int]
    #search for index: not efficient, but does the job
    while True:
        if history[index][1] > timestamp-0.005: return index-1
        index += 1

def get_label(index):
    labels = history[index-2:index+1]
    #return str(labels[2][1])
    #if past two labels are more frequent -> output these
    if labels[0][0] == labels[1][0]: return labels[0][0]
    #else most recent one
    return labels[2][0]


#find starting frame in cap
sync_time = 1506005146.24 #timestamp of sync

sync_time_real = 9 #secs in real video
sync_frame = int(fps*sync_time_real) #frame in real video

delay_secs = 1
starting_frame = sync_frame - fps*delay_secs
starting_time = sync_time - delay_secs


#render starting at starting frame
count = 0
while count < starting_frame:
    ret, frame = cap.read()
    count += 1

current_time = starting_time

while True:
    ret, frame = cap.read()
    if not ret: break
    index = get_index_of(current_time)
    label = get_label(index)
    
    cv2.fillConvexPoly(frame, np.array([[0,h],[0,h-110],[400,h-110],[400,h]]), (0,0,0))
    cv2.putText(frame, label, (30,h-30), cv2.FONT_HERSHEY_SIMPLEX, 2, (250,250,250), 3)
    out.write(frame)
    current_time += 1.0/fps

out.release()
print('Converting...')
call(['avconv', '-i', '%s.avi'%filename, '-c:v', 'libx264', '-c:a', 
        'copy', '%s.mp4'%filename])
call(['rm', '%s.avi'%filename])
print("Recorded and converted")

cap.release()



