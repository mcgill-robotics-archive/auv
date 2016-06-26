#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Sonar Scan Preprocessor.

Listens to PointCloud scan objects and eliminates all points below a certain
intensity or below a certain radius.
"""

import rospy
import numpy as np
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32


__author__ = "Jana Pavlasek, Dihia Idrici"

# Preprocessing constants.
MIN_INTENSITY = 90
MIN_RADIUS = 1
MAX_RADIUS  = 6
MAX_RECENTERED_RADIUS = 0.4
MIN_RECENTERED_RADIUS = 0.0
MIN_NUMBER_POINT = 40


def preprocess(scan):
    """Preprocess scan."""
    cloud = PointCloud()

    cloud.header.frame_id = scan.header.frame_id
    cloud.header.stamp = rospy.get_rostime()
    cloud.points = []

    channel = ChannelFloat32()
    channel.name = "intensity"
    channel.values = []

    """
    Index refers to the position of data point within a scan!
    Because we want to go through each point one at a time, we need index to
    increase by 1 each time.If the point param:eters are in accordance with the
    requirements it will fill the filtered cloud.
    """

    index = 0
    for intensity in scan.channels[0].values:
        # Distance from center to point.
        point = scan.points[index]
        radius = ((point.x)**2 + (point.y)**2)**0.5
        # Only add point is requirements are let.
        if intensity > MIN_INTENSITY and radius > MIN_RADIUS and radius < MAX_RADIUS:
            channel.values.append(intensity)
            cloud.points.append(scan.points[index])
        index += 1
    print ("the final index value is %d" % index)

    """
    Filter a second time the new pointcloud considering the density of point.

    Newcenter corresponds to a point i about which we look at the
    agglomeration of other points.

    OtherPoint correspond to a point j in the viscinity of point i. If enough
    point j are located within a given radius of point i, then point i remains.
    """

    pointradius = []
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
            if R > MIN_RECENTERED_RADIUS and R < MAX_RECENTERED_RADIUS:
                radiusesfromi.append(R)
            j += 1

        pointradius.append([Newcenter.x, Newcenter.y, len(radiusesfromi)])
        # reset radiusesfromi for the next point i
        radiusesfromi = [] 
        i += 1
        
    # Transform point radius into a readable numpy array
    X = np.array(pointradius)

    k = 0 
    w = 0 
    for x in range(0, len(cloud.points)):
    	V = X[k][2]
    	if V < MIN_NUMBER_POINT:
    	    # print V
            del cloud.points[w]
            del channel.values[w]
            # print channel.values[w]
            k += 1
        else: 
            k += 1
            w += 1

    print ("the filtered number of point is %d") % w
    cloud.channels = [channel]
    scan_pub.publish(cloud)

if __name__ == '__main__':
    # Initialize publishers and subscribers.
    rospy.init_node("scan_preprocessor_third")
    slice_sub = rospy.Subscriber("full_scan", PointCloud,
                                 preprocess, queue_size=1)
    scan_pub = rospy.Publisher("python_filtered_scan", PointCloud, queue_size=1)

    rospy.spin()

