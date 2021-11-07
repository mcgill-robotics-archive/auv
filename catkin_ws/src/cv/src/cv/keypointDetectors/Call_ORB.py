#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
from cv.msg import CvTarget
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from dynamic_reconfigure.server import Server
from cv.cfg import laneDetectorParamsConfig

'''
	This Finds a target image using ORB keypoints and 
	k nearest neighbors matching. 
	I first tried to make it a subscriber, but 
	the callback is called so fast that it doesn't have
	Time to complete and it throws errors. 
	This needs to be an action for sure
'''
class ORBDetectorServer():

    def __init__(self):
        self.pub    = rospy.Publisher('cv/down_cam_target', 
        	                          CvTarget, 
        	                          queue_size=1)

        self.bridge = CvBridge()
        #self.sub    = rospy.Subscriber("/camera_front_1/image_raw", 
        #	                           Image, 
        #	                           self.callback)

        self.targetFound        = False
        self.smoothQueue        = deque([])
        self.debugimgs          = True #When true, show debugging images
        #This should NOT be hard codedhere.
        #TODO: Make this a parameter
        self.img_target         = cv2.imread('DarkTower.png',0) # Query Image
        self.orb                = cv2.ORB_create()
        print("starting ORB Detector")



    def execute(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        print('inside Execute')

        xReturn = None
        yReturn = None

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.orb.detectAndCompute(self.img_target,None)
        print(len(kp1))
        kp2, des2 = self.orb.detectAndCompute(img,None)
        print(len(kp2))

        FLANN_INDEX_KDTREE = 0
        index_params       = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params      = dict(checks = 50)
        FLANN_INDEX_LSH    = 6

        index_params = dict(algorithm = FLANN_INDEX_LSH,
                           table_number      = 6, # 12
                           key_size          = 12,     # 20
                           multi_probe_level = 1) #2
                           
        flann   = cv2.FlannBasedMatcher(index_params, search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        MIN_MATCH_COUNT = 2        
        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask     = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = self.img_target.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)

            img = cv2.polylines(img,[np.int32(dst)],True,255,3, cv2.LINE_AA)

        else:
            print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
            matchesMask = None
            
        draw_params = dict(matchColor       = (0,255,0), # draw matches in green color
                           singlePointColor = None,
                           matchesMask      = matchesMask, # draw only inliers
                           flags            = 2)

        img3 = cv2.drawMatches(self.img_target,kp1,img,kp2,good,None,**draw_params)

        scaling = 0.5
        small   = cv2.resize(img3, (0, 0), fx=scaling, fy=scaling)
        cv2.imshow(label, small)
        #cv2.moveWindow(label, dispx,dispy);
        cv2.waitKey(1)
        
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
            self.laneFound = True
        else:
            #If no correct lane is found we want to make sure to turn the servo off
            #  print("no lane") --------------------------------------------------------------------------
            msg = CvTarget()
            #This makes sure the servo turns off
            msg.probability.data = 0.0
            self.pub.publish(msg)
            #also clear the queue
            self.smoothQueue = deque([])
            self.laneFound = False

            #small = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
            #cv2.imshow("Final Image", small)
            #cv2.waitKey(5)

    def stop(self):
        self.sub.unregister()

    def smoothPoint(self,xVal,yVal):
        if len(self.smoothQueue) < 4:
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

    def dyn_reconf_cb(self, config, level):
    	'''
    	%This is the dynamic reconfigure callback. 
    	This function updates all of the 
    	reconfigurable parameters. It is fed into the Server
    	and updates everything defined in the .cfg file
    	'''

    	#rospy.loginfo('{}'.format(config))

    	#These are values for color masking
    	self.hueLowerBound = config.hueLowerBound
    	self.hueUpperBound = config.hueUpperBound
    	self.satLowerBound = config.satLowerBound
    	self.satUpperBound = config.satUpperBound
        self.valLowerBound = config.valLowerBound
        self.valUpperBound = config.valUpperBound
        #These values are for the hough line detection

        #These are values for gaussian blurring

    	return config


    def display_debug_image(self,debug,img,label,scaling,dispx,dispy):
    	'''
    	This function displays the input image. 
    	I call it a lot for debugging, so the function
    	is convenient.
    	Only does anything if debug == True
    	#TODO: make window scaling less crappy
    	'''
    	if (debug == True):
    		small = cv2.resize(img, (0, 0), fx=scaling, fy=scaling)
    		cv2.imshow(label, small)
        	#cv2.moveWindow(label, dispx,dispy);
        	cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('ORBDetector',anonymous=True)
    #bla = ORBDetector()
    #srv = Server(laneDetectorParamsConfig, bla.dyn_reconf_cb)
    server = ORBDetectorServer()
    rospy.spin()
