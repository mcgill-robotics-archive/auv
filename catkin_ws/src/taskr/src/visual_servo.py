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
    # TODO: find a way to run node inside this object
    def __init__(self, target):
        rospy.loginfo("Initializing VisualServo action with target {}".format(target))
        self.target = target

    def start(self, server, feedback_msg):
        rospy.loginfo("Starting VisualServo action")

def tracked_obj_callback(box):
    if box.confidence > 0.50:
        box_center_x = box.x + box.width/2
        box_center_y = box.y + box.height/2

        # aimed x and y define the pixel on the image to which we aim the
        # center of the bounding box
        aimed_x = IMAGE_CENTER_X
        aimed_y = IMAGE_CENTER_Y
        target_width = BUOY_DIAMETER
        target_height = BUOY_DIAMETER

        x_pixels_to_target = box_center_x - aimed_x
        y_pixels_to_target = box_center_y - aimed_y

        # e.g. 100 px to target center / 50 px for the tracked buoy * 0.2m buoy width
        # = 0.4 m to target center
        x_dist_to_target = float(x_pixels_to_target) / float(box.width) * target_width
        y_dist_to_target = float(y_pixels_to_target) / float(box.height) * target_height

        point = Point()
        point.x = 0.0  #TODO: get x (surge) distance based on size of bb
        #TODO: assign real distance to y and z based on size of bb
        point.y = x_dist_to_target
        point.z = y_dist_to_target
        pub.publish(point)
    else:
        # TODO: try to find it again (after some time?)
        point = Point()
        point.x = 0.0
        point.y = 0.0
        point.z = 0.0
        pub.publish(point)

if __name__ == '__main__':
    global pub, sub
    rospy.init_node('visual_servo')
    # TODO: parametrize target name
    pub = rospy.Publisher('red_buoy', Point, queue_size=10)
    sub = rospy.Subscriber(
        '/tld_tracked_object', BoundingBox, tracked_obj_callback)
    rospy.spin()
