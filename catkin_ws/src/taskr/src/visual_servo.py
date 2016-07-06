#!/usr/bin/env python
from geometry_msgs.msg import Point

import rospy

from tld_msgs.msg import BoundingBox


BUOY_DIAMETER = 0.20
IMAGE_WIDTH = 1296
IMAGE_HEIGHT = 964
IMAGE_CENTER_X = IMAGE_WIDTH / 2
IMAGE_CENTER_Y = IMAGE_HEIGHT / 2
DIST_ERROR = 0.1


class VisualServo(object):
    def __init__(self, target):
        rospy.loginfo("Initializing VisualServo action with target {}"
                      .format(target))
        self.target = target
        self.pub = rospy.Publisher(target, Point, queue_size=10)
        self.sub = rospy.Subscriber(
            '/tld_tracked_object', BoundingBox, self.tracked_obj_callback)

        # aimed x and y define the pixel on the image to which we aim the
        # center of the bounding box
        self.aimed_x = IMAGE_CENTER_X
        self.aimed_y = IMAGE_CENTER_Y
        self.target_width = BUOY_DIAMETER
        self.target_height = BUOY_DIAMETER

    def start(self, server, feedback_msg):
        rospy.loginfo("Starting VisualServo action")

        # while loop with rate to check if no box for some time
        while True:
            rospy.logdebug("Visual Servoing")
            rospy.sleep(0.05)
            continue

    def tracked_obj_callback(self, box):
        if box.confidence > 0.50:
            self.last_frame_time = rospy.get_rostime()

            box_center_x = box.x + box.width / 2
            box_center_y = box.y + box.height / 2

            x_pixels_to_target = box_center_x - self.aimed_x
            y_pixels_to_target = box_center_y - self.aimed_y

            # e.g. 100 px to target center / 50 px for the tracked buoy * 0.2m
            # buoy width = 0.4 m to target center
            x_dist_to_target = float(x_pixels_to_target) / float(box.width) * self.target_width
            y_dist_to_target = float(y_pixels_to_target) / float(box.height) * self.target_height

            point = Point()
            point.x = 0.0  # TODO: get x (surge) distance based on size of bb
            # TODO: assign real distance to y and z based on size of bb
            point.y = x_dist_to_target
            point.z = y_dist_to_target
            self.pub.publish(point)
        else:
            # TODO: try to find it again (after some time?)
            point = Point()
            point.x = 0.0
            point.y = 0.0
            point.z = 0.0
            self.pub.publish(point)
