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

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        MIN_SIZE_GREEN = rospy.get_param("cv/roulette/green_cnt_size", 3000)

        #blur img
        img = cv2.GaussianBlur(img,(7,7),2)
        #convert to hvt
        img_hvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #TODO: adapt values as adequate
        lower_green = np.array([70, 160, 30])
        upper_green = np.array([90, 255, 255])
        mask = cv2.inRange(img_hvt, lower_green, upper_green)

        im2,cnt,hier = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)

        if len(cnt) != 0:
            # find contour with biggest area
            biggestConts = heapq.nlargest(2,cnt, key = cv2.contourArea)
            centers = []
            for c in biggestConts:
                #check contour size
                if (cv2.contourArea(c) < MIN_SIZE_GREEN):
                    #TODO: ADD ABORT CONDITIONS
                    print("No contour of sufficient size found")
                    break
                # draw in blue the contours that were found
                cv2.drawContours(img, [c], -1, (255,0,0),3,8)
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centers.append((cX,cY))

            for cX,cY in centers:
                cv2.circle(img,(cX,cY),10,(0,0,255),-1,-1)

            if len(centers) == 0:
                print("No relevant green contour found!")
                center = None
            else:
                if len(centers) == 1:
                    center = centers[0]
                elif len(centers) >= 2:
                    center = (int((centers[0][0] + centers[1][0]) / 2),int((centers[0][1] + centers[1][1]) / 2) )

                cv2.circle(img, center, 10, (0, 255, 255), -1, -1)

            if (center != None):
                msg = CvTarget()
                msg.gravity.x = center[0]
                msg.gravity.y = center[1]
                msg.gravity.z = 0
                msg.probability.data = 1.0
                self.pub.publish(msg)
                self.rouletteFound = True
            else:
                self.rouletteFound = False

        small = cv2.resize(img,(0,0),fx=0.5,fy=0.5)
        cv2.imshow("RouletteImage",small)
        cv2.waitKey(20)

    def getRouletteFound(self):
        return self.rouletteFound
    def stop(self):
        self.sub.unregister()