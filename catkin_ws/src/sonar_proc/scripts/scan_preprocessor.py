#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Sonar Scan Preprocessor.

Listens to PointCloud scan objects and eliminates all points below a certain
intensity or below a certain radius.
"""

import rospy
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

__author__ = "Jana Pavlasek"

# Preprocessing constants.
MIN_INTENSITY = 5
MIN_RADIUS = 1


def preprocess(scan):
    """Preprocess scan."""
    cloud = PointCloud()

    cloud.header.frame_id = "robot"
    cloud.header.stamp = rospy.get_rostime()
    cloud.points = []

    channel = ChannelFloat32()
    channel.name = "intensity"
    channel.values = []

    index = 0
    for intensity in scan.channels[0].values:
        # Distance from center to point.
        point = scan.points[index]
        radius = ((point.x)**2 + (point.y)**2)**0.5
        # Only add point is requirements are met.
        if intensity > MIN_INTENSITY and radius > MIN_RADIUS:
            channel.values.append(intensity)
            cloud.points.append(scan.points[index])
        index += 1

    cloud.channels = [channel]
    scan_pub.publish(cloud)


if __name__ == '__main__':
    # Initialize publishers and subscribers.
    rospy.init_node("scan_preprocessor")
    slice_sub = rospy.Subscriber("/full_scan", PointCloud,
                                 preprocess, queue_size=1)
    scan_pub = rospy.Publisher("filtered_scan", PointCloud, queue_size=1)

    rospy.spin()
