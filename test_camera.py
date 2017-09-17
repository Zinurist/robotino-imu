#!/usr/bin/env python
import camera
import cv2

"""
Test to see if the USB camera is recognized and usable.
"""

cv2.namedWindow("preview")

if vc.isOpened(): # try to get the first frame
    frame = camera.get_val()
    rval = True
else:
    rval = False

while rval:
    cv2.imshow("preview", frame)
    frame = camera.get_val()
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break
cv2.destroyWindow("preview")
