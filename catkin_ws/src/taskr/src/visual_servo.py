#!/usr/bin/env python

import rospy
from tld_msgs.msg import BoundingBox
from geometry_msgs.msg import Point

BUOY_DIAMETER = 0.20
IMAGE_WIDTH = 1296
IMAGE_HEIGHT = 964
IMAGE_CENTER_X = IMAGE_WIDTH/2
IMAGE_CENTER_Y = IMAGE_HEIGHT/2

class VisualServo(object):
    def __init__(self, torpedo):
        pass

    def start(self, server, feedback_msg):
        pass

def tracked_obj_callback(box):
	if box.confidence > 0.50:
		box_center_x = box.x + box.width/2
		box_center_y = box.y + box.height/2
		point = Point()
		point.x = 0.0  #TODO: get x (surge) speed based on size of bb
		#TODO: assign real distance to y and z based on size of bb
		point.y = box_center_x - IMAGE_CENTER_X
		point.z = box_center_y - IMAGE_CENTER_Y
		pub.publish(point)

if __name__ == '__main__':
	global pub, sub
	rospy.init_node('visual_servo')
	pub = rospy.Publisher('/buoy/red', Point, queue_size=10)
	sub = rospy.Subscriber(
	    '/tld_tracked_object', BoundingBox, tracked_obj_callback)
	rospy.spin()
