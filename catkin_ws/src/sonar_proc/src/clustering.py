#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy

from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg._ColorRGBA import ColorRGBA


__author__ = "Dihia Idrici, Jana Pavlasek"

"""
    mean_shift.py uses this class, to generate a new MarkerArray
    containing markers (or clusters) for each input from /filtered_scan topic

    Depending on the angle range we will select for our sonar scan, the
    markers lifetime (self.m.lifetime.sec) will have to be adjusted
"""


class Clustering(object):

    def __init__(self, pointcloud):
        self.points = self._populate(pointcloud)
        self.intensities = pointcloud.channels[0].values
        self.numpy_points = numpy.array(self.points)
        self.size = len(self.points)

    def _populate(self, pointcloud):
        points = [[point.x, point.y] for point in pointcloud.points]
        return points

    def _new_average(self, previous_average, count, update):
        return (previous_average*count + update) / (count + 1)

    # TODO: combine get_average_intensities and get_sizes such that
    # we only need to loop through the labels once

    # labels are meant to be a map, by index,
    # to the channel given in self.intensities
    def get_average_intensities(self, labels, number_of_clusters):
        ave = [[0, 0] for x in range(number_of_clusters)]
        for label, intensity in zip(labels, self.intensities):
            # don't include points that aren't clustered
            if label == -1:
                continue

            ave[label][0] = self._new_average(
                ave[label][0],
                ave[label][1],
                intensity)
            ave[label][1] += 1
        return [x[0] for x in ave]

    # get the number of points in each cluster
    def get_sizes(self, labels, number_of_clusters):
        sizes = [0 for x in range(number_of_clusters)]
        for label in labels:
            sizes[label] += 1
        return sizes

    def make_markers(self, cluster_centers):
        marker_array = MarkerArray()
        marker_colours = ["red", "blue", "green", "yellow", "cyan"]

        stamp = rospy.get_rostime()
        for i, cluster_center in enumerate(cluster_centers):
            marker = Marker()

            marker.header.frame_id = "robot"
            marker.header.stamp = stamp

            # Marker namespace.
            marker.ns = "sonar"
            marker.id = i
            marker.type = Marker.POINTS

            # use up all the colours
            # if there's none left, make it white
            if not marker_colours:
                colour = "white"
            else:
                colour = marker_colours.pop()
            marker.color = colours[colour]

            # Alpha must be set or marker will be invisible.
            marker.scale.x = 0.20
            marker.scale.y = 0.20
            marker.scale.z = 0.20

            marker.lifetime.secs = 6

            # Points list holds a single point which corresponds
            # to the cluster location.
            point = Point()
            point.x = cluster_center[0]
            point.y = cluster_center[1]

            marker.points = [point]
            marker_array.markers.append(marker)

        return marker_array

colours = {
    "red" : ColorRGBA(1,0,0,1),
    "green": ColorRGBA(0,1,0,1),
    "blue": ColorRGBA(0,0,1,1),
    "yellow": ColorRGBA(1,1,0,1),
    "cyan": ColorRGBA(0,1,1,1),
    "white": ColorRGBA(1,1,1,1)
}
