#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import heapq
from cv.msg import CvTarget
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TorpedoDetector():

    def __init__(self):
        self.MIN_TARGET_SIZE = rospy.get_param('cv/detectors/torpedo/min_target_size',1000)
        self.pub = rospy.Publisher('cv/front_cam_target', CvTarget, queue_size=1)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/camera_front1/image_raw", Image, self.callback)
        rospy.loginfo("starting TorpedoDetector")

        self.targetFound = False

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # convert to hvt
        img_hvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # TODO: adapt values as adequate
        lower_red1 = np.array([0, 50, 0])
        upper_red1 = np.array([15, 255, 255])

        lower_red2 = np.array([165, 50, 0])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(img_hvt, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hvt, lower_red2, upper_red2)

        mask = np.bitwise_or(mask1, mask2)
        #dilate to get a more visible target
        mask = cv2.dilate(mask, (5, 5), iterations=3)

        im2, cnt, hier = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        if len(cnt) != 0:
            # find contour with biggest area
            biggestConts = heapq.nlargest(1, cnt, key=cv2.contourArea)

            target = biggestConts[0]

            # debug
            print("Biggest Found Contour: {}".format(cv2.contourArea(target)))

            # check contour size
            if (cv2.contourArea(target) < MIN_TARGET_SIZE):
                # TODO: ADD ABORT CONDITIONS
                print("No contour of sufficient size found")

                foundCenter = (None,None)
                self.targetFound = False
            else:
                #debug
                cv2.drawContours(mask, target, -1, (255, 255, 0), 1, 8)

                #calculate Target
                M = cv2.moments(target)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                foundCenter = (cX,cY)
                self.targetFound = True

                #debug
                cv2.circle(mask, (cX, cY), 10, (0, 0, 255), -1, -1)

        else: #no contours found
            print("No countour found")
            self.targetFound = False
            foundCenter = (None,None)

        # TODO: additional checks to make sure the targets are all valid

        msg = CvTarget()
        msg.gravity.x = foundCenter[0]
        msg.gravity.y = foundCenter[1]
        msg.gravity.z = 0
        msg.probability.data = 1.0
        self.pub.publish(msg)

        # debug
        small = cv2.resize(mask, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow("TorpedoImage", small)
        cv2.waitKey(10)

    def getTargetFound(self):
        return self.targetFound

    def stop(self):
        self.sub.unregister()
