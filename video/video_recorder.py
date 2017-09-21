import cv2
import time
from subprocess import call
import numpy as np
from threading import Thread

cap = cv2.VideoCapture(1)
w,h = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
fps = int(cap.get(cv2.CAP_PROP_FPS))
fourcc = cv2.VideoWriter_fourcc(*'MJPG')

def get_label():
    return str(time.time())


filename = 'recording_labelled'
out = cv2.VideoWriter('%s.avi' % filename, fourcc, fps, (w,h), isColor=True)

try:
    while True:
        ret, frame = cap.read()
        label = get_label()
        
        cv2.fillConvexPoly(frame, np.array([[0,h],[0,h-110],[400,h-110],[400,h]]), (0,0,0))
        cv2.putText(frame, label, (30,h-30), cv2.FONT_HERSHEY_SIMPLEX, 2, (250,250,250), 3)
        out.write(frame)
except KeyboardInterrupt:
    out.release()
    print('Converting...')
    call(['avconv', '-i', '%s.avi'%filename, '-c:v', 'libx264', '-c:a', 
            'copy', '%s.mp4'%filename])
    call(['rm', '%s.avi'%filename])
    print("Recorded and converted")

    cap.release()



