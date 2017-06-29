import os
import xml.etree.ElementTree as ET
import numpy as np
import cv2
import pywt as pt

img = cv2.imread('plane.jpg',0)
#wavelet transform function
def w2d(imArray, mode='haar', level=1):
    #convert to float
    imArray =  np.float32(imArray)
    imArray /= 255;
    # compute coefficients
    coeffs=pt.wavedec2(imArray, mode, level=level)
    
    #Process Coefficients
    coeffs_H=list(coeffs)
    coeffs_H[0] *= 0;
    
    # reconstruction
    imArray_H=pt.waverec2(coeffs_H, mode);
    imArray_H *= 255;
    imArray_H = np.uint8(imArray_H)
    return imArray_H

cap = cv2.VideoCapture("02.MOV")
while not cap.isOpened():
    cap = cv2.VideoCapture("02.MOV")
    cv2.waitKey(1000)
    print "Wait for the header"

pos_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
count = 0
previous_points = []
current_points = []
real_track = []
save_list = []
while True:
    #flag, frame = cap.read()
    flag, current_frame = cap.read()
    current_points = []
    real_track = []
    preserve = current_frame.copy()
    if count == 0:
        previous_frame = current_frame

    if flag:
        # The frame is ready and already captured
        current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        previous_frame_gray = cv2.cvtColor(previous_frame, cv2.COLOR_BGR2GRAY)
        current_frame = w2d(current_frame,'haar',1)
        previous_frame = w2d(previous_frame,'haar',1)
        frame_diff = cv2.absdiff(current_frame_gray,previous_frame_gray)
        frame_diff = cv2.blur(frame_diff,(6,6))
        frame_diff = cv2.erode(frame_diff, None, iterations=2)
        frame_diff = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
        #frame_diff = cv2.dilate(frame_diff, None, iterations=2)
        cnts = cv2.findContours(frame_diff.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        # loop over the contours
        for c in cnts:
            # compute the bounding box for the contour, draw it on the frame,
            # and update the text
            (x, y, w, h) = cv2.boundingRect(c)
            if w <= 10 or h <= 10:
                continue
            if w >= 300 or h >= 300:
                continue
            current_points.append((x,y,w,h))

        for points_c in current_points:
            current_x = points_c[0]
            current_y = points_c[1]
            current_w = points_c[2]
            current_h = points_c[3]
            for points_p in previous_points:
                previous_x = points_p[0]
                previous_y = points_p[1]
                eu_length = np.sqrt((current_x - previous_x)**2 + (current_y - previous_y)**2)
                if eu_length <= 20:
                    real_track.append((current_x,current_y,current_w,current_h))

        for point in real_track:
            x = point[0]
            y = point[1]
            w = point[2]
            h = point[3]
            cv2.rectangle(preserve, (x, y), (x + w, y + h), (0, 255, 0), 2)

        #cv2.imshow('frame diff ', frame_diff)
        cv2.imshow('frame diff ', preserve)
        if len(real_track) == 1:
            print "hello"
            point = real_track[0]
            save_list.append(point)
            cv2.imwrite("plane-" + str(count) + ".jpg", preserve)
        
        if count != 0:
            previous_frame = current_frame

        previous_points = current_points
        if len(real_track) > 0:
            previous_points = real_track
        print count, "frames"
        # cv2.imshow('video', frame)
        #pos_frame = cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
        #print str(pos_frame)+" frames"
    count += 1
    if cv2.waitKey(1) & 0xff == 27:
        break
    if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
        # If the number of captured frames is equal to the total number of frames,
        # we stop
        break

saved = np.array(save_list)
saved = saved.reshape((-1,4))
np.savetxt('test.txt', saved)


'''
https://stackoverflow.com/questions/24536552/how-to-combine-pywavelet-and-opencv-for-image-processing
https://stackoverflow.com/questions/5707353/how-to-extend-pywavelets-to-work-with-n-dimensional-data
'''







