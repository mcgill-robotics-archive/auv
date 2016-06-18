#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Sonar Scan Preprocessor.

Listens to PointCloud scan objects and eliminates all points below a certain
intensity or below a certain radius.
"""

import rospy
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32


__author__ = "Jana Pavlasek, Dihia Idrici"

# Preprocessing constants.
MIN_INTENSITY = 100
MIN_RADIUS = 1
MAX_RECENTERED_RADIUS = 0.5
MIN_RECENTERED_RADIUS = 0.1
MIN_NUMBER_POINT = 40


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
    # print ("the number of point is:")
    # print len(cloud.points)
    # print len(channel.values)
    # print channel.values
    # print cloud.points

    """
    Filter a second time the new pointcloud considering the density of point.

    Newcenter corresponds to a point i about which we look at the
    agglomeration of other points.

    OtherPoint correspond to a point j in the viscinity of point i. If enough
    point j are located within a given radius of point i, then point i remains.
    """

    i = 0
    for x in range(0, len(cloud.points)):
        Newcenter = cloud.points[i]   # Cloud.point array with ,x= y= z=, array
        intensitypoint = channel.values[i]
        xNewCenter = Newcenter.x
        yNewCenter = Newcenter.y
        radiusesfromi = []
        # print ("the value of i is %d") % i
        j = 0
        for x in cloud.points:
            OtherPoint = cloud.points[j]
            xOtherPoint = OtherPoint.x
            yOtherPoint = OtherPoint.y

            xdifference = abs(xNewCenter - xOtherPoint)
            ydifference = abs(yNewCenter - yOtherPoint)
            R = ((xdifference)**2 + (ydifference)**2)**0.5
            if R < MAX_RECENTERED_RADIUS and R > MIN_RECENTERED_RADIUS:
                radiusesfromi.append(R)
            j += 1
        # print ("The length of radius")
        # print len(radiusesfromi)
        # print radiusesfromi

        # After the loop is completed for a given point i
        if len(radiusesfromi) < MIN_NUMBER_POINT:
            # print channel.values[i]
            # print channel.values
            # print ("i remains the same and is equal %d") % i
            # print ("1 - the value of j is %d") % j
            del cloud.points[i]
            del channel.values[i]
            radiusesfromi = []  # Reset the array
            j = 0  # reset j
            # print ("1 - the value of j is %d") % j
            # print ("The new radius array")
            # print radiusesfromi
            # print len(cloud.points)
            # No i +=1 because then data will have shifted left by one position
        else:
            i += 1  # Consider the next point i
            # print ("1 - the value of j is %d") % j
            j = 0  # reset j
            # print ("1 - the value of j is %d") % j
            # print ("1 - i increases by 1 and equals %d") % i
            radiusesfromi = []  # Reset the array
            # print ("The new radius array")
            # print radiusesfromi
            # print len(cloud.points)

    print ("the value of i is %d") % i

    cloud.channels = [channel]
    # print len(channel.values)
    # print len(cloud.points)
    # print channel.values

    scan_pub.publish(cloud)
    # sample_pub.publish(i)

if __name__ == '__main__':
    # Initialize publishers and subscribers.
    rospy.init_node("scan_preprocessor_third")
    slice_sub = rospy.Subscriber("full_scan", PointCloud,
                                 preprocess, queue_size=1)
    scan_pub = rospy.Publisher("filtered_scan", PointCloud, queue_size=1)

    # So that we know exactly the number of sample to
    # consider in bandwidth calculation
    # sample_pub = rospy.Publisher("n_sample", Int32, queue_size=1)

    rospy.spin()
