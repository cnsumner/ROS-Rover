#!/usr/bin/env python
from picamera.array import PiRGBArray
from picamera import PiCamera
import rospy
from std_msgs.msg import Int16, Byte
import numpy as np
import time
import cv2
from math import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def pathfinder():
    rospy.init_node('imgCapture', anonymous=True, disable_signals=True)
    pub = rospy.Publisher("imgFrame", Image, queue_size=10)

    cap = cv2.VideoCapture("/home/pi/video.mp4")

    while ~cap.isOpened():
        print('No good.')
        time.sleep(1)

    time.sleep(0.1)

    avgCenter = 639/2
    samples = 0

    fps = 30

    count = 0

    bridge = CvBridge()
    try:
        while (True):
            ret, frame = cap.read()

            img = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(img);
            cap.truncate(0)

    except:
        pass
    cam.close()

if __name__ == '__main__':
    try:
        pathfinder()
    except rospy.ROSInterruptException:
        pass
