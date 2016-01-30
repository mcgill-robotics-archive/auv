#!/usr/bin/env python

from sensor_msgs.msg import PointCloud, LaserScan
import numpy as np
import math
import rospy


def sonar_angle_to_rad(angle):
    """Converts angles in units of 1/16th of a gradian to radians.
    Args:
        angle: Angle in 1/16th of a gradian.
    Returns:
        Angle in radians.
    """
    return float(angle) * np.pi / 3200


def pointcloud_to_laserscan(cloud):
    # Get cloud ... cloud =

    if len(cloud.channels) == 1:
        channel = cloud.channels[0]
    else:
        raise Exception

    maxValue = 0
    index = 0
    for x in range(len(channel.values)):
        if (channel.values[x] > maxValue):
            maxValue = channel.values[x]
            index = x

    point = cloud.points[index]  # x, y of maxValue (z = 0)
    angle = math.atan2(point.y, point.x)  # value from -pi to +pi
    angle += np.pi  # value from 0 to 2pi

    scan = LaserScan()

    # Set Header
    scan.header = cloud.header

    # Set angular range.
    scan.angle_min = angle
    scan.angle_max = angle + 1/16
    scan.angle_increment = 1/16

    scan.time_increment = 0
    scan.scan_time = 1 / 22.28

    scan.range_min = 0
    scan.range_max = 5

    # Range
    # num_of_headings = 1
    # scan.ranges = [math.hypot(point.x, point.y)]
    scan.ranges = [2.5, 2.5]
    scan.intensities = [100, 100]
    # scan.intensities = [255, 255]

    return scan

    """
    scan = LaserScan()

    scan.header = cloud.header

    scan.angle_min = - math.pi
    scan.angle_max = math.pi
    scan.angle_increment = 2 * math.pi / 200

    scan.time_increment = 8.6 / 200
    scan.scan_time = 8.6

    scan.range_min = 0
    scan.range_max = 30

    scan.ranges = [2.5 for x in range(200)]
    scan.intensities = [100 for x in range(200)]
    return scan
    """
    """
    # Get latest N slices.
    queued_slices = sorted(
        self.slices, key=lambda x: x.timestamp
    )[-1 * queue:]

    # Set time and time increments.
    scan.time_increment = (
        queued_slices[-1].timestamp - queued_slices[0].timestamp
        #TIME INCREMENT BETWEEN LAST AND FIRST SLICE
    ).total_seconds() if len(queued_slices) > 1 else 0
    scan.scan_time = (
        queued_slices[-1].timestamp - queued_slices[0].timestamp
    ).total_seconds() #TIME INCREMENT BETWEEN LAST AND FIRST SLICE

    # Header.
    scan.header.frame_id = frame
    self.time += rospy.Duration.from_sec(scan.time_increment)
    scan.header.stamp = self.time

    # Set angular range.
    scan.angle_min = sonar_angle_to_rad(0)
    scan.angle_max = sonar_angle_to_rad(6400)
    # scan.angle_increment = sonar_angle_to_rad(self.steps) ??

    # Determine ranges and intensities.
    num_of_headings = 6400 / self.steps # One heading?
    scan.range_max = self.range # Max range
    scan.range_min = 0.1
    scan.ranges = [0 for i in range(num_of_headings)]
    scan.intensities = [0 for i in range(num_of_headings)]
    for scan_slice in queued_slices:
        index = scan_slice.heading / self.steps
        scan.ranges[index] = scan_slice.max
        scan.intensities[index] = scan_slice.max_intensity

    return scan

    """


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    laser = pointcloud_to_laserscan(data)
    pub.publish(laser)
    # Add additional


if __name__ == '__main__':
    rospy.init_node('pointcloud_to_laserscan')

    rospy.Subscriber(
        "/tritech_micron/scan", PointCloud, callback, queue_size=1)
    pub = rospy.Publisher("~laserscan", LaserScan, queue_size=1)
    rospy.spin()
