#!/usr/bin/env python

from sensor_msgs.msg import PointCloud, LaserScan
import math
import rospy

__author__ = "Brennan Nichyporuk"


def pointcloud_to_laserscan(cloud):

    if len(cloud.channels) == 1:
        channel = cloud.channels[0]
    else:
        raise Exception

    maxValue = 0
    index = 0

    """The PointCloud had a range of ~10 meters. Ignore the first fifth of all
    data points (ignore any readings with a range of less than 2 meters)."""
    for x in range(len(channel.values)/5, len(channel.values)):
        if (channel.values[x] > maxValue):
            maxValue = channel.values[x]
            index = x

    arbitraryThreshold = 50
    # If no point meets this threshold, use the last point in the data set.
    if maxValue < arbitraryThreshold:
        index = len(channel.values) - 1

    point = cloud.points[index]
    angle = math.atan2(point.y, point.x)
    length = math.hypot(point.x, point.y)

    scan = LaserScan()

    scan.header = cloud.header

    scan.angle_min = angle
    scan.angle_max = angle
    scan.angle_increment = 0

    scan.time_increment = 0
    scan.scan_time = 0

    scan.range_min = 1
    scan.range_max = 11

    """I am unsure why two values are required for the ranges list below. One
    value doesn't seem to work"""
    scan.ranges = [length, length]

    return scan


def callback(data):
    laser = pointcloud_to_laserscan(data)
    pub.publish(laser)


if __name__ == '__main__':
    rospy.init_node('pointcloud_to_laserscan')
    rospy.Subscriber(
        "/tritech_micron/scan", PointCloud, callback, queue_size=1)
    pub = rospy.Publisher("laserscan", LaserScan, queue_size=1)
    rospy.spin()

