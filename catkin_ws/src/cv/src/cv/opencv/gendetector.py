#!/usr/bin/env python

import cv2
import rospy
import math
import numpy as np

#from cv.msg import CvTarget
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

class GenericDetector(object):

	def __init__(self, image_topic,
				 target_topic,
				 dark_pub_topic,
				 dark_sub_topic):
		self.sub = rospy.Subscriber(image_topic, Image, self.process)
		#self.pub = rospy.Publisher(pub_topic, CvTarget, queue_size=1)
		self.darkPub = rospy.Publisher(dark_pub_topic, Image, queue_size=1)
		self.darkSub = rospy.Subscriber(
			dark_sub_topic, BoundingBoxes, self.darkCb)

		self.bridge = CvBridge()
		#self.target = CvTarget()

		self.color = False
		self.display = True

	def process(self, data):
		try:
			# Note: We use bgr8 encoding because it is the OpenCV standard
			# for working with color images
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerror(e)

		# Display Original Image -----------------------------------------------
		if self.display:
			cv2.imshow('Original',img)

		# Run Preprocessing on the Image ---------------------------------------
		pimg = self.preprocess(img)
		if self.display:
			cv2.imshow('Preprocessed', pimg)

		## FOR COLOR THRESHOLDING
		if self.color:
			# Create HSV Copy --------------------------------------------------
			hsv = cv2.cvtColor(pimg, cv2.COLOR_BGR2HSV)
			if self.display:
				cv2.imshow('HSV Variant', hsv)

			# Threshold Based on Color -----------------------------------------
			mask = self.cthreshold(hsv, 'orange')
			if self.display:
				cv2.imshow('mask', mask)

		## FOR NEAR BLACK OR NEAR WHITE THRESHOLDING
		else:
			# Create Grayscale Copy --------------------------------------------
			gray = cv2.cvtColor(pimg, cv2.COLOR_BGR2GRAY)
			if self.display:
				cv2.imshow('Grayscale', gray)

			# Threshold Based on Intensity -------------------------------------
			mask = self.bwthreshold(gray, 'black')
			if self.display:
				cv2.imshow('mask', mask)

			# Publish Grayscale image for Darknet ------------------------------
			recol = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
			mlimg = self.bridge.cv2_to_imgmsg(recol, encoding="bgr8")
			self.darkPub.publish(mlimg)

		# Process Contours -----------------------------------------------------
		cont = self.contours(mask)
		if self.display:
			cimg = cv2.drawContours(pimg, cont, -1, (0, 0, 255), 3)
			cv2.imshow('Contours', cimg)

		cv2.waitKey(1)


	def preprocess(self, img):
		# Median blur to reduce noise
		img = cv2.medianBlur(img, 7)
		# (Small) Gaussian blur to smooth transformation
		img = cv2.GaussianBlur(img, (3, 3), 0)
		# Downsample the image using nearest neighbor method
		img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
		# Decrease the blue channel
		img[:, :, 0] = img[:, :, 0] * 0.5
		# Increase the red channel
		#img[:, :, 2] = img[:, :, 2] * 1.5
		# Ensure number values are still in range
		#img = np.clip(img, 0, 255)

		return img

	def cthreshold(self, img, key):
		lower = {
			'black': np.array([0, 0, 0]),
			'orange': np.array([30, 150, 200]),
			'orange2': np.array([38, 150, 200]),
			'bins-test': np.array([25, 20, 160])
		}

		upper = {
			'black': np.array([255, 255, 10]),
			'orange': np.array([35, 255, 255]),
			'orange2': np.array([43, 255, 255]),
			'bins-test': np.array([35, 150, 255])
		}

		mask = cv2.inRange(img, lower[key], upper[key])
		mask = cv2.medianBlur(mask, 5)

		return mask

	def bwthreshold(self, img, key):
		mask = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
			cv2.THRESH_BINARY, 11, 4)

		return mask

	def contours(self, mask):
		_, cont, _ = cv2.findContours(
			mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
		fcont = []
		for c in cont:
			area = cv2.contourArea(c)

			if ((cv2.contourArea(c) > 200) and
				(cv2.contourArea(c) < 300000)):

			## DO OTHER CHECKS HERE FOR CONTOUR PROPERTY FILTERING

				rect = cv2.minAreaRect(c)
				box = cv2.boxPoints(rect)
				box = np.int0(box)

				fcont.append(box)

		return fcont

	def darkCb(self, data):
		boxes = data.bounding_boxes

if __name__ == '__main__':
	rospy.init_node('TestDetector')
	detector = GenericDetector('camera_down/image_rect_color', 'test_target',
		'darknet_img', '/darknet_ros/bounding_boxes')
	rospy.spin()
