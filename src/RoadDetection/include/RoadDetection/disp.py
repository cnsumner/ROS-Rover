#!/usr/bin/env python
from picamera.array import PiRGBArray
from picamera import PiCamera
import rospy
from std_msgs.msg import Int16
import numpy as np
import time
import cv2
from math import *

def pathfinder():
    pub = rospy.Publisher('Steering', Int16, queue_size=10)
    rospy.init_node('RoadDetection', anonymous=True)

    #cap = cv2.VideoCapture(0)
    cam = PiCamera()
    cam.resolution = (640, 360)
    cam.framerate = 5
    cap = PiRGBArray(cam, size=(640, 360))
    cap.truncate(0)

    time.sleep(0.1)

    #cap.set(3, 640)
    #cap.set(4, 360)
    #cap.set(5, 10)

    #while~cap.isOpened():
    #    print('No good.')
    #    time.sleep(1)

    cv2.namedWindow('frame', flags=cv2.WINDOW_AUTOSIZE)

    avgCenter = 639/2
    samples = 0

    fps = 30

    count = 0
    for frame in cam.capture_continuous(cap, format="bgr", use_video_port=True):
       
        start_time = time.time()
        #ret, frame = cap.read()
        image = frame.array

        cv2.imshow('frame', image)

        cap.truncate(0)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        pathfinder()
    except rospy.ROSInterruptException:
        pass
