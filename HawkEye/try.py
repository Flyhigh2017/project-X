import os
import xml.etree.ElementTree as ET
import numpy as np
import cv2
import pywt as pt

img = cv2.imread('plane.jpg',0)
#read in video
cap = cv2.VideoCapture("test.MOV")
while not cap.isOpened():
    cap = cv2.VideoCapture("test.MOV")
    cv2.waitKey(1000)
    print "Wait for the header"

pos_frame = cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
count = 0
while True:
    #flag, frame = cap.read()
    flag, current_frame = cap.read()
    if count == 0:
        previous_frame = current_frame

    if flag:
        # The frame is ready and already captured
        current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        previous_frame_gray = cv2.cvtColor(previous_frame, cv2.COLOR_BGR2GRAY)

        frame_diff = cv2.absdiff(current_frame_gray,previous_frame_gray)
    
        cv2.imshow('frame diff ',frame_diff)
        if count != 0:
            previous_frame = current_frame
        print count, "frames"
        # cv2.imshow('video', frame)
        #pos_frame = cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
        #print str(pos_frame)+" frames"

    else:
        # The next frame is not ready, so we try to read it again
        cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, pos_frame-1)
        print "frame is not ready"
        # It is better to wait for a while for the next frame to be ready
        cv2.waitKey(1000)
    count += 1
    if cv2.waitKey(10) == 27:
        break
    if cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES) == cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT):
        # If the number of captured frames is equal to the total number of frames,
        # we stop
        break
'''
https://stackoverflow.com/questions/24536552/how-to-combine-pywavelet-and-opencv-for-image-processing
https://stackoverflow.com/questions/5707353/how-to-extend-pywavelets-to-work-with-n-dimensional-data
'''







