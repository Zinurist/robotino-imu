#!/usr/bin/env python

import cv2
import numpy as np
import scipy as sp

vc = cv2.VideoCapture(0)

"""
An easy interface to get data from the camera for recording.
@return an image from the camera
"""
def get_val():
    rval, frame = vc.read()
    frame = np.dot(frame[...,:3], [0.299, 0.587, 0.114])
    frame = sp.misc.imresize(frame, 0.3)
    return frame

