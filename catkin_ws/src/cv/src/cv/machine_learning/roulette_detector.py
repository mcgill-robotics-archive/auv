#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import heapq
from cv.msg import CvTarget
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RouletteDetector():

    def __init__(self):
        self.pub = rospy.Publisher('cv/down_cam_target', CvTarget, queue_size=1)
        self.bridge = CvBridge()
        rospy.loginfo("starting roulette detector")
        self.sub = rospy.Subscriber("/camera_down/image_raw", Image, self.callback)

        self.rouletteFound = False
        self.MIN_SIZE_GREEN = rospy.get_param("cv/roulette/green_cnt_size", 3000)
        self.MIN_SIZE_RED = rospy.get_param("cv/roulette/red_cnt_size", 3000)

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #blur img
        img = cv2.GaussianBlur(img,(7,7),2)
        #convert to hvt
        img_hvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        redCenter = self.findRedCenter(img_hvt,img)
        greenCenter = self.findGreenCenter(img_hvt,img)


        if redCenter is None or greenCenter is None:
            print("No green AND red found.")
            msg = CvTarget()
            msg.probability.data = 0.0
            self.pub.publish(msg)
            self.rouletteFound = False
            return
        else:
            rouletteCenter = (int((redCenter[0] + greenCenter[0]) / 2) , int((redCenter[1] + greenCenter[1])) / 2)
            cv2.circle(img, rouletteCenter, 20, (0, 255, 255), -1, -1)

            small = cv2.resize(img, (0, 0), fx=0.4, fy=0.4)
            cv2.imshow("SUPER", small)
            cv2.waitKey(20)

            msg = CvTarget()
            msg.gravity.x = rouletteCenter[0]
            msg.gravity.y = rouletteCenter[1]
            msg.gravity.z = 0
            msg.probability.data = 1.0
            self.pub.publish(msg)
            self.rouletteFound = True


    def findRedCenter(self,img_hvt,img):

        lower_red1 = np.array([0, 50, 0])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([165, 50, 0])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(img_hvt, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hvt, lower_red2, upper_red2)

        mask = np.bitwise_or(mask1, mask2)

        im2, cnt, hier = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        if len(cnt) != 0:
            # find contour with biggest area
            biggestConts = heapq.nlargest(2,cnt, key = cv2.contourArea)
            centers = []
            for c in biggestConts:
                #check contour size
                if (cv2.contourArea(c) < self.MIN_SIZE_RED):
                    #TODO: ADD ABORT CONDITIONS
                    print("No contour of sufficient size found")
                    break
                # draw in red the contours that were found
                cv2.drawContours(img, [c], -1, (0,0,255),3,8)
                #calculate centroid
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centers.append((cX,cY))

            #for cX,cY in centers:
                #cv2.circle(img,(cX,cY),10,(0,0,255),-1,-1)

            if len(centers) == 0:
                print("No relevant red contour found!")
                center = None
            else:
                if len(centers) == 1:
                    center = centers[0]
                elif len(centers) >= 2:
                    center = (int((centers[0][0] + centers[1][0]) / 2),int((centers[0][1] + centers[1][1]) / 2) )

                cv2.circle(img, center, 10, (0, 0, 255), -1, -1)
        else:
            center = None
        return center


    def findGreenCenter(self,img_hvt,img):
        #TODO: adapt values to competition
        #lower_green = np.array([87, 140, 0])
        #upper_green = np.array([93, 255, 150])
        lower_green = np.array([70, 160, 30])
        upper_green = np.array([90, 255, 255])

        # green contours
        mask = cv2.inRange(img_hvt, lower_green, upper_green)

        _, cnt, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        if len(cnt) != 0:
            # find contours with biggest area
            biggestConts = heapq.nlargest(2, cnt, key=cv2.contourArea)
            centers = []
            for c in biggestConts:
                # check contour size
                if (cv2.contourArea(c) < self.MIN_SIZE_GREEN):
                    continue

                # draw in green the contours that were found
                cv2.drawContours(img, [c], -1, (0, 255, 0), 3, 8)

                #get centroid of contour
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centers.append((cX, cY))

            #for cX, cY in centers:
            #    cv2.circle(img, (cX, cY), 10, (0, 255, 0), -1, -1)

            if len(centers) == 0:
                center = None
            else:
                if len(centers) == 1:
                    center = centers[0]
                elif len(centers) >= 2:
                    center = (int((centers[0][0] + centers[1][0]) / 2), int((centers[0][1] + centers[1][1]) / 2))

                cv2.circle(img, center, 10, (0, 255, 0), -1, -1)

        else:
            center = None

        return center

    def getRouletteFound(self):
        return self.rouletteFound
    def stop(self):
        self.sub.unregister()
