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
        self.scan_config = {}
        self.num_slices = None  # The number of slices in a full scan.

    def make_config(self, data):
        """Updates scan cofigurations when configuration is published."""
        self.scan_config["left_limit"] = data.left_limit
        self.scan_config["left_limit"] = data.right_limit
        self.scan_config["step"] = data.step

        total = abs(self.scan_config["left_limit"] - self.scan_config["left_limit"])

        # For continuous data, default to 2pi.
        if total == 0 or data.continuous:
            total = 2 * math.pi

        self.num_slices = total / self.scan_config["step"]

        # In the case of 49.999999 for example, must round before casting to
        # avoid data loss.
        self.num_slices = int(round(self.num_slices, 0))

    def stitch(self, data):
        """Callback for stitch. If scan is not full, add a slice, otherwise
        publish the full scan."""
        # If there is no config data, nothing can be done.
        if not self.scan_config:
            rospy.loginfo("No sonar config is present, cannot stitch")
            return

        # Check if scan is full.
        if not self.full():
            self.clouds.append(data)
        else:
            # Publish current scan.
            self.scan_pub.publish(self.to_full_scan(data.header.frame_id))

            # Clear the scan data scan.
            self.clouds = []

    def full(self):
        """Returns True the scan is full and False otherwise."""
        return len(self.clouds) == self.num_slices

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
