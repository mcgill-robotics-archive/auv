#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Sonar Scan Stitcher.

This listens to PointCloud slices and stitches them into a full scan for
analysis.
"""

import math
import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from tritech_micron.msg import TritechMicronConfig


__author__ = "Jana Pavlasek, Anass Al-Wohoush, Max Krogius"

scan = None
scan_config = {}


class Scan(object):

    """Scan."""

    def __init__(self, range=None, steps=None, num_bins=None, left_limit=None,
                 right_limit=None, clockwise=None):
        """Constructs Scan object."""
        self.range = range
        self.steps = steps
        self.num_bins = num_bins
        self.left_limit = left_limit
        self.right_limit = right_limit
        self.clockwise = clockwise
        self.clouds = []
        self.time = rospy.get_rostime()
        self.full = False

    """def empty(self):
        # Returns whether the scan is empty.
        return len(self.clouds) == 0 """

    def add(self, cloud_slice):  # cloud_slice = data from stitch
        """Adds a slice to the scan.

        Args:
            scan_slice: Slice of the scan
        """
        theta = round(self.theta(cloud_slice.points[10]), 4)
        # gives 4 decimal of theta

        # Update full variable if the end is reached.
        if not self.empty() and (theta == round(self.right_limit, 4) or
                                 theta == round(self.left_limit, 4)):
            self.full = True

        self.clouds.append(cloud_slice)

        print self.clouds
        # print theta  # all of those points are at the same angle

    def to_full_scan(self, frame):  # frame = data from stitch
        """Publishes the sonar data as a point cloud.

        Args:
            frame: Name of sensor frame.

        Returns:
            sensor_msgs.msg.PointCloud.
        """
        cloud = PointCloud()

        cloud.header.frame_id = frame
        cloud.header.stamp = rospy.get_rostime()
        cloud.points = [
            point for cloud in self.clouds
            for point in cloud.points
        ]

        channel = ChannelFloat32()
        channel.name = "intensity"
        channel.values = [
            intensity for cloud in self.clouds
            for channel in cloud.channels
            for intensity in channel.values
        ]

        cloud.channels = [channel]
        return cloud

    def theta(self, point):
        """Calculates the angle of a point, in the range 0 to 2pi, where 0 is
        the positive y axis.

        Args:
            point: Point32 message.
        """
        theta = math.atan2(point.y, point.x)

        # Wrap values around to be between 0 and 2pi.
        if theta < 0:
            theta = 2*math.pi + theta

        return theta

def make_config(data):
    """Updates scan cofigurations when configuration is published."""
    if (not scan_config or
            scan_config["left_limit"] != data.left_limit or
            scan_config["right_limit"] != data.right_limit or
            scan_config["range"] != data.range or
            scan_config["num_bins"] != data.nbins or
            scan_config["steps"] != data.step or
            scan_config["clockwise"] != data.scanright):
        scan_config["left_limit"] = data.left_limit
        scan_config["right_limit"] = data.right_limit
        scan_config["range"] = data.range
        scan_config["num_bins"] = data.nbins
        scan_config["steps"] = data.step
        scan_config["clockwise"] = data.scanright


def stitch(data):
    """Callback for stitch. If scan is not full, add a slice, otherwise
    publish the full scan and reinitialize."""
    # If a scan has not been initialized, make one.
    if not scan:
        if scan_config:
            global scan
            scan = Scan(**scan_config)
        else:
            return  # No config, do nothing for now.

    # Check if scan is full.
    if not scan.full:
        scan.add(data)
    else:
        # Publish current scan.
        frame = rospy.get_param("~frame", "odom")
        scan_pub.publish(scan.to_full_scan(frame))

        # Reinitialize a new scan.
        global scan
        scan = Scan(**scan_config)
        scan.add(data)


if __name__ == '__main__':
    # Initialize publishers and subscribers.
    rospy.init_node("scan_stitcher")
    # Data from the subscription are used by make_config
    config_sub = rospy.Subscriber("/tritech_micron/config",
                                  TritechMicronConfig,
                                  make_config, queue_size=1)
    # Data from the subscription are used by stitch
    slice_sub = rospy.Subscriber("filtered_slice_scan", PointCloud,
                                 stitch, queue_size=1)
    scan_pub = rospy.Publisher("full_filtered_scan", PointCloud, queue_size=1)

    rospy.spin()
