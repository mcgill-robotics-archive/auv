#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Sonar Scan Preprocessor.

Listens to PointCloud scan slice objects and eliminates all points
below a certain intensity or below a certain radius.
"""

import rospy
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from matplotlib import pyplot

__author__ = "Modified by Dihia Idrici from Jana Pavlasek Original version"

# Preprocessing constants.
# MIN_INTENSITY = 5
MIN_RADIUS = 1  # Data desired after a radius 1m about the sonar


def preprocess_slice(scan):
    """Preprocess scan."""

    # Declare PointCloud
    cloud = PointCloud()
    # Fillinf PointCloud header
    cloud.header.frame_id = "robot"  # Frame in which the scan was taken
    cloud.header.stamp = rospy.get_rostime()
    # Each point32 should be interpreted as a 3D point in the
    # frame given in the header
    cloud.points = []

    # Declare ChannelFloat32() called intensity to the PointCloud
    cloud.channels = ChannelFloat32()
    cloud.channels.name = "intensity"
    # Should contain the element of the point cloud
    cloud.channels.values = []

    # Populate PointCloud and the intensity Channel with data
    index = 0
    radia = []  # fill radius array
    for intensity in scan.channels[0].values:
        # Channels being the top name of the intensity array
        # when you print cloud
        # Distance from center to point.
        point = scan.points[index]
        radius = ((point.x)**2 + (point.y)**2)**0.5
        # Only add point if radius requirements are met.
        if radius > MIN_RADIUS:
            # Filling the cloud with intensity (channel data) and point data
            # from PointCloud
            cloud.channels.values.append(intensity)
            cloud.points.append(scan.points[index])
            radia.append(float(radius))
        index += 1

    distribution(radia, cloud.channels.values)
    filtered_slice_pub.publish(cloud)
    # print radia
    # print cloud


def distribution(r, intensity):

    # Graphical visualization of the intensity distribution
    pyplot.title("Sonar slice intensity distribution")
    pyplot.plot(r, intensity, 'b')
    pyplot.xlabel("Radius")
    pyplot.ylabel("Intensity")
    pyplot.plot(r, intensity, 'b')
    pyplot.show()


if __name__ == '__main__':
    rospy.init_node("scan_slice_filter")
    slice_sub = rospy.Subscriber("/tritech_micron/scan",
                                 PointCloud,
                                 preprocess_slice, queue_size=1)
    filtered_slice_pub = rospy.Publisher("filtered_slice_scan",
                                         PointCloud, queue_size=1)

    rospy.spin()
