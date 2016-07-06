#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Sonar Scan Stitcher.

This listens to PointCloud slices and stitches them into a full scan for
analysis.
"""

import math
import rospy
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from tritech_micron.msg import TritechMicronConfig


__author__ = "Jana Pavlasek, Anass Al-Wohoush, Max Krogius"


class ScanStitcher(object):

    """ScanStitcher object."""

    def __init__(self):
        """Constructs Scan object."""
        rospy.init_node("scan_stitcher")

        self.config_sub = rospy.Subscriber("/tritech_micron/config", TritechMicronConfig,
                                           self.make_config, queue_size=1)
        self.slice_sub = rospy.Subscriber("/tritech_micron/scan", PointCloud,
                                          self.stitch, queue_size=1)
        self.scan_pub = rospy.Publisher("full_scan", PointCloud, queue_size=1)

        self.clouds = []
        self.full = False
        self.scan_config = {}

    def make_config(self, data):
        """Updates scan cofigurations when configuration is published."""
        if (not self.scan_config or
                self.scan_config["left_limit"] != data.left_limit or
                self.scan_config["right_limit"] != data.right_limit or
                self.scan_config["range"] != data.range or
                self.scan_config["num_bins"] != data.nbins or
                self.scan_config["steps"] != data.step or
                self.scan_config["clockwise"] != data.scanright):
            self.scan_config["left_limit"] = data.left_limit
            self.scan_config["right_limit"] = data.right_limit
            self.scan_config["range"] = data.range
            self.scan_config["num_bins"] = data.nbins
            self.scan_config["steps"] = data.step
            self.scan_config["clockwise"] = data.scanright

    def stitch(self, data):
        """Callback for stitch. If scan is not full, add a slice, otherwise
        publish the full scan."""
        # If there is no config data, nothing can be done.
        if not self.scan_config:
            rospy.loginfo("No sonar config is present, cannot stitch")
            return

        # Check if scan is full.
        if not self.full:
            self.add(data)
        else:
            # Publish current scan.
            self.scan_pub.publish(self.to_full_scan(data.header.frame_id))

            # Clear the scan data scan.
            self.clouds = []
            self.full = False

    def empty(self):
        """Returns True the scan is empty and False otherwise."""
        return len(self.clouds) == 0

    def add(self, cloud_slice):
        """Adds a slice to the scan.

        Args:
            scan_slice: Slice of the scan.
        """
        theta = round(self.theta(cloud_slice.points[10]), 4)

        # Update full variable if the end is reached.
        if not self.empty() and (theta == round(self.scan_config["right_limit"], 4) or
                                 theta == round(self.scan_config["left_limit"], 4)):
            self.full = True

        self.clouds.append(cloud_slice)

    def to_full_scan(self, frame):
        """Returns the sonar data as a point cloud.

        Args:
            frame: Name of sonar frame.

        Returns:
            sensor_msgs.msg.PointCloud.
        """
        cloud = PointCloud()

        cloud.header.frame_id = frame
        cloud.header.stamp = rospy.get_rostime()
        cloud.points = [
            pt for cld in self.clouds
            for pt in cld.points
        ]

        channel = ChannelFloat32()
        channel.name = "intensity"
        channel.values = [
            intensity for cld in self.clouds
            for ch in cld.channels
            for intensity in ch.values
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
            theta = 2 * math.pi + theta

        return theta


if __name__ == '__main__':
    ScanStitcher()
    rospy.spin()
