#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Sonar Scan Preprocessor.

Listens to PointCloud scan objects and eliminates all points below a certain
intensity or below a certain radius.
"""

import rospy
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

__author__ = "Jana Pavlasek, Dihia Idrici"

# Preprocessing constants.
MIN_INTENSITY = 75
MIN_RADIUS = 1
MIN_RECENTERED_RADIUS = 2
MIN_NUMBER_POINT = 150


def preprocess(scan):
    """Preprocess scan."""
    cloud = PointCloud()

    cloud.header.frame_id = "robot"
    cloud.header.stamp = rospy.get_rostime()
    cloud.points = []

    channel = ChannelFloat32()
    channel.name = "intensity"
    channel.values = []

    """
    Index refers to the position of data point within a scan!
    Because we want to go through each point one at a time, we need index to
    increase by 1 each time.If the point parameters are in accordance with the
    requirements it will fill the filtered cloud.
    """

    index = 0
    for intensity in scan.channels[0].values:
        # Distance from center to point.
        point = scan.points[index]
        radius = ((point.x)**2 + (point.y)**2)**0.5
        # Only add point is requirements are let.
        if intensity > MIN_INTENSITY and radius > MIN_RADIUS:
            channel.values.append(intensity)
            cloud.points.append(scan.points[index])
        index += 1
    # print ("the final index value is %d" % index)

    """
    We filter a second time the new pointcloud adding another constraint:
    the density around the point.

    Newcenter corresponds to a point i about which we are looking at the
    agglomeration of other points.

    OtherPoint correspond to a point j in the viscinity of point i. If enough
    point j are located within a given radius of point i, then point i remains.
    """
    i = 0
    for x in cloud.points:
        # print ("the value of i is: %d" % i) --> IT WORKED
        Newcenter = cloud.points[i]   # Cloud.point array with ,x= y= z=, array
        xNewCenter = Newcenter.x
        yNewCenter = Newcenter.y
        radiusesfromi = []

        j = 0
        for x in cloud.points:
            # print ("the value of j is %d" % j)  --> IT WORKED

            OtherPoint = cloud.points[j]
            xOtherPoint = OtherPoint.x
            yOtherPoint = OtherPoint.y

            xdifference = abs(xNewCenter - xOtherPoint)
            ydifference = abs(yNewCenter - yOtherPoint)
            R = ((xdifference)**2 + (ydifference)**2)**0.5

            if R < MIN_RECENTERED_RADIUS and R != 0:
                radiusesfromi.append(R)
            j += 1

        # print ("the final value of j is %d" % j)  --> IT WORKED
        j = 0  # reset j for the new i
        # print ("j has been reset %d" % j)  --> IT WORKED
        if len(radiusesfromi) < MIN_NUMBER_POINT:
            del cloud.points[i]
            del channel.values[i]
            radiusesfromi = []  # Reset the array
            # No i +=1 because then data will have shifted left by one position
        else:
            i += 1  # Consider the next point i
            radiusesfromi = []  # Reset the array


    cloud.channels = [channel]
    scan_pub.publish(cloud)


if __name__ == '__main__':
    # Initialize publishers and subscribers.
    rospy.init_node("scan_preprocessor")
    slice_sub = rospy.Subscriber("full_scan", PointCloud,
                                 preprocess, queue_size=1)
    scan_pub = rospy.Publisher("filtered_scan", PointCloud, queue_size=1)

    rospy.spin()
