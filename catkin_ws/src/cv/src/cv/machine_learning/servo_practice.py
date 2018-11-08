#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
from cv.msg import CvTarget
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

class ServoPracticeF():

    def __init__(self):
        self.pub = rospy.Publisher('cv/front_cam_target', CvTarget, queue_size=1)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/camera_front_1/image_raw", Image, self.callback)
        self.smoothQueue = deque([])

    def callback(self,data):
        try:
            #rospy.loginfo("retrieving image")
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # This is orange_filter:
        MIN_CONTOUR_SIZE = rospy.get_param("/cv/lanes/orange_cnt_size", 10000)
        img_hvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 50, 50])
        upper_orange = np.array([30, 255, 255])
        mask = cv2.inRange(img_hvt, lower_orange, upper_orange)

        im2, cnt, hier = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        if len(cnt) == 0:
            print("No lane of sufficient size found")
            return

        else:
            #Contours were found
            # find contour with biggest area
            biggestCont = max(cnt, key=cv2.contourArea)
            # check contour size

            if (cv2.contourArea(biggestCont) < MIN_CONTOUR_SIZE):
                print("No lane of sufficient size found")
                return
            else:
                # draw in blue the contours that were found
                cv2.drawContours(img, [biggestCont], -1, (255, 0, 0), 3, 8)

                # create emptyImage for mask
                contourMask = np.zeros(img.shape, np.uint8)
                cv2.drawContours(contourMask, [biggestCont], -1, (255, 255, 255), cv2.FILLED, 8)

                M = cv2.moments(biggestCont)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                foundCenter = (cX, cY)

                xReturn = cX
                yReturn = cY

                if (xReturn != None):
                    #new
                    xReturn,yReturn = self.smoothPoint(xReturn,yReturn)

                    cv2.circle(img, (int(xReturn), int(yReturn)), 10, (255, 0, 255), -1)

                    msg = CvTarget()
                    msg.gravity.x = xReturn
                    msg.gravity.y = yReturn
                    msg.gravity.z = 0
                    msg.probability.data = 1.0
                    self.pub.publish(msg)


                small = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
                cv2.imshow("image", small)
                cv2.waitKey(5)


    def smoothPoint(self,xVal,yVal):
        if len(self.smoothQueue) < 3:
            self.smoothQueue.append((xVal,yVal))
        else:
            self.smoothQueue.popleft()
            self.smoothQueue.append((xVal, yVal))

        meanX = 0.0
        meanY = 0.0
        for x,y in self.smoothQueue:
            meanX += x
            meanY += y

        meanX = meanX / len(self.smoothQueue)
        meanY = meanY / len(self.smoothQueue)

        return meanX, meanY

class ServoPracticeD():

    def __init__(self):
        self.pub = rospy.Publisher('cv/down_cam_target', CvTarget, queue_size=1)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/camera_down/image_raw", Image, self.callback)
        self.smoothQueue = deque([])

    def callback(self,data):
        try:
            #rospy.loginfo("retrieving image")
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # This is orange_filter:
        MIN_CONTOUR_SIZE = rospy.get_param("/cv/lanes/orange_cnt_size", 10000)
        img_hvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 50, 50])
        upper_orange = np.array([30, 255, 255])
        mask = cv2.inRange(img_hvt, lower_orange, upper_orange)

        im2, cnt, hier = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        if len(cnt) == 0:
            print("No lane of sufficient size found")
            return

        else:
            #Contours were found
            # find contour with biggest area
            biggestCont = max(cnt, key=cv2.contourArea)
            # check contour size

            if (cv2.contourArea(biggestCont) < MIN_CONTOUR_SIZE):
                print("No lane of sufficient size found")
                return
            else:
                # draw in blue the contours that were found
                cv2.drawContours(img, [biggestCont], -1, (255, 0, 0), 3, 8)

                # create emptyImage for mask
                contourMask = np.zeros(img.shape, np.uint8)
                cv2.drawContours(contourMask, [biggestCont], -1, (255, 255, 255), cv2.FILLED, 8)

                M = cv2.moments(biggestCont)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                foundCenter = (cX, cY)

                xReturn = cX
                yReturn = cY

                if (xReturn != None):
                    #new
                    xReturn,yReturn = self.smoothPoint(xReturn,yReturn)

                    cv2.circle(img, (int(xReturn), int(yReturn)), 10, (255, 0, 255), -1)

                    msg = CvTarget()
                    msg.gravity.x = xReturn
                    msg.gravity.y = yReturn
                    msg.gravity.z = 0
                    msg.probability.data = 1.0
                    self.pub.publish(msg)


                small = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
                cv2.imshow("image", small)
                cv2.waitKey(5)


    def smoothPoint(self,xVal,yVal):
        if len(self.smoothQueue) < 3:
            self.smoothQueue.append((xVal,yVal))
        else:
            self.smoothQueue.popleft()
            self.smoothQueue.append((xVal, yVal))

        meanX = 0.0
        meanY = 0.0
        for x,y in self.smoothQueue:
            meanX += x
            meanY += y

        meanX = meanX / len(self.smoothQueue)
        meanY = meanY / len(self.smoothQueue)

        return meanX, meanY
