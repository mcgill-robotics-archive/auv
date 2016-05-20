#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Sonar Scan Preprocessor.

Listens to PointCloud scan slice objects and eliminates all points
below a certain intensity or below a certain radius.
"""

import rospy
import math
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from matplotlib import pyplot

__author__ = "Modified by Dihia Idrici from Jana Pavlasek Original scan_preprocessor"

# Preprocessing constants.
# MIN_INTENSITY = 5
MIN_RADIUS = 1  # Data desired after a radius 1m about the sonar


def preprocess_slice(scan):
    """Preprocess scan.

    Scan: Data from /tritech_micron/scan to which our node (scan_slice_filter)
          subscribed. The scan is for

    I added few check points to verify if the data made sence. Do not mind them:
        - thet, theta
        - print radia
        - print cloud
    """

    # Declare PointCloud
    cloud = PointCloud()
    # Fill PointCloud header
    # Should be frame in which the scan was taken
    cloud.header.frame_id = "robot"
    cloud.header.stamp = rospy.get_rostime()
    # Each point32 should be interpreted as a 3D point in the
    # frame given in the header
    # Contains x, y and z coordinates. However z=0 in our case
    cloud.points = []

    # Declare ChannelFloat32() called intensity to the PointCloud
    cloud.channels = ChannelFloat32()
    cloud.channels.name = "intensity"
    # Should contain the intensity element of the point cloud
    cloud.channels.values = []

    # Populate PointCloud and the intensity Channel with data
    index = 0
    radia = []  # fill radius array
    thet = []
    for intensity in scan.channels[0].values:
        # Channels: Name by default of the intensity array
        point = scan.points[index]
        radius = ((point.x)**2 + (point.y)**2)**0.5
        theta = math.atan2(point.y, point.x)
        # Only add point if radius requirements are met.
        if radius > MIN_RADIUS:
            # Filling the cloud with intensity (channel data) and point data
            # from PointCloud
            cloud.channels.values.append(intensity)
            cloud.points.append(scan.points[index])
            radia.append(float(radius))
            thet.append(float(theta))
        index += 1
        # print theta / this was meant to prove all of the theta are the same
        # Which is the assumption to make for a given beam/scan slice
        # well the 7 first decimal: Good accuracy!

    # print thet  # same 8 first decimal
    # print radia
    # print cloud
    # print cloud.channels.values
    distribution(radia, cloud.channels.values)
    # slice_filter(cloud.points, cloud.channels.values)
    filtered_slice_pub.publish(cloud)


def distribution(r, intensity):

    # Graphical visualization of the intensity distribution
    pyplot.title("Sonar slice intensity distribution")
    pyplot.plot(r, intensity, 'b')
    pyplot.xlabel("Radius")
    pyplot.ylabel("Intensity")
    pyplot.plot(r, intensity, 'b')
    pyplot.show()

# def slice_filter(coordinates, intensity):



if __name__ == '__main__':
    rospy.init_node("scan_slice_filter")
    slice_sub = rospy.Subscriber("/tritech_micron/scan",
                                 PointCloud,
                                 preprocess_slice, queue_size=1)
    filtered_slice_pub = rospy.Publisher("filtered_slice_scan",
                                         PointCloud, queue_size=1)

    rospy.spin()
