#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Byte
from sensor_msgs.msg import Image
import numpy as np
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from math import *

class dummy:

    pub = rospy.Publisher('motors', Byte, queue_size=10)

    bridge = CvBridge()

    avgCenter = 639/2
    samples = 0
    fps = 30
    count = 0

    def pathfinder(self):

        self.listener()

        self.pub.publish(0x00)
        cv2.destroyAllWindows()

    def callback(self,data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        ##############################

        start_time = time.time()
        #ret, image = cap.read()

        template = image[250:500, 200:330]
        #template = frame
        rowAvg = np.average(template, axis=0)
        avg = np.uint8(np.average(rowAvg, axis=0))

        #roi = image[100:359, 0:639]
        roi = image[200:359, 0:639]

        mask = cv2.inRange(roi, avg - [60, 60, 60], avg + [60, 60, 60])
        mask = np.bitwise_and(cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY), mask)
        #mask = np.bitwise_and(roi, mask)

        blur = cv2.blur(roi, (5,5))
        canny = cv2.Canny(blur, 20, 100, 10)
        canny = cv2.dilate(canny, (5,5))

        #lines = cv2.HoughLines(canny, 1, np.pi/180, 200);
        lines = cv2.HoughLines(canny, 1, np.pi/180, 200)
        #lines = []

        canny = cv2.cvtColor(canny,cv2.COLOR_GRAY2RGB)

        try:
            range = lines.shape[0]
        except AttributeError:
            range = 0

        x3 = 0
        y3 = y4 = 100
        x4 = 639
        cv2.line(roi, (x3, y3), (x4, y4), (255, 0, 0))
        cv2.line(roi, (640/2, 0), (640/2, 500), (255, 0, 0))

        intersections = []
        for i in xrange(range):
            for rho, theta in lines[i]:
                angle = theta * 180 / np.pi

                if not(100 > angle > 80) and not(10 > angle):
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    cv2.line(roi, (x1, y1), (x2, y2), (0, 255, 0))

                    intersectionX = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4))
                    intersectionY = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4))
                    if (intersectionX < 0):
                        intersectionX = 0
                    elif (intersectionX > 639):
                        intersectionX = 639
                    intersections.append((intersectionX, intersectionY))

        intersections.sort(key=lambda point: point[0])
        intersections.insert(0, (0, 200))
        intersections.insert(0, (639, 200))
        mid = (0, 0)
        prev = -1
        for x, y in intersections:
            if prev == -1:
                prev = x
            else:
                d = x - prev
                prev = x
                if d > mid[0]:
                    mid = (d, (x,y))
            cv2.circle(roi, (x, y), 4, (0, 0, 255), -1)

        #samples += 1
	print("img1")
        if not(mid == (0, 0)):
	    print("img0")
            center = mid[1][0] - mid[0]/2
            self.avgCenter = (self.avgCenter*1 + center)/(1 + 1)

            cv2.circle(roi, (self.avgCenter, y3), 4, (255, 0, 255), -1)

            value = int(self.avgCenter - 640/2)
            rospy.loginfo(value)
            if value < -30:
                self.pub.publish(0x48)
                print("img2")
            elif value > 30:
                self.pub.publish(0x42)
                print("img3")
            else:
                self.pub.publish(0x40)
                print("img4")
            print("img5")

        #fps = time.time() - start_time
        #rospy.loginfo(fps)

        ##############################

        #cv2.imshow('frame', roi)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    return

    def listener(self):
        rospy.init_node('RoadDetection', anonymous=True, disable_signals=True)
        rospy.Subscriber("imgFrame", Image, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        d = dummy()
        d.pathfinder()
    except rospy.ROSInterruptException:
        pass
