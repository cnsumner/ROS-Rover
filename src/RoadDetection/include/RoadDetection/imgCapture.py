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

    cam = PiCamera()
    cam.resolution = (640, 360)
    cam.framerate = 3
    cap = PiRGBArray(cam, size=(640, 360))
    cap.truncate(0)

    time.sleep(0.1)

    avgCenter = 639/2
    samples = 0

    fps = 30

    count = 0

    bridge = CvBridge()
    try:
        for frame in cam.capture_continuous(cap, format="bgr", use_video_port=True):
            img = bridge.cv2_to_imgmsg(frame.array, "bgr8")
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
