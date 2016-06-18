#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""MeanShift Clustering Test.

Inspiration from plot_mean_shift.py example from SciKit Learn:

http://scikit-learn.org/stable/auto_examples/cluster/plot_mean_shift.html#example-cluster-plot-mean-shift-py

Meanshiftthird.py considers the x and y position, as well as the intensity 
each point from the filtered_scan as parameters to consider in meanshift

Publishes:
    /cluster_markers:

Subscribes:
    /filtered_scan: PointCloud message containing filtered sonar scan

Currently in the process of increasing the scan filtering level!

"""

import rospy
import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

__author__ = "Dihia Idrici, Jana Pavlasek"

X = None


def populate(data):
    """ The algorithm starts by making a copy of the original fitered data
    set from the topic "full_filtered_scan"
    and freezing the original points. The copied points
    are shifted against the original frozen points."""

    # Define array named pts which we fill with the data points
    # from a given sonar scan
    pts = []

    # zip allows us to iterate over two lists in parallel
    for point, val in zip(data.points, data.channels[0].values):
        intensity = val  # Range [0, 255]
        # Fill points array with x, y and intensity value
        # we ommit z as it is always zero (2D representation)
        pts.append([point.x, point.y, intensity])

    # Transform data into usuable Numpy arrays.
    global X
    X = np.array(pts)  # Matrix with three column
    print X

    cluster()


def cluster():

    """Clustering with MeanShift"""

    # Bandwidth has to be estimated
    bandwidth = estimate_bandwidth(X, quantile=0.2)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
    ms.fit(X)
    print X
    labels = ms.labels_
    cluster_centers = ms.cluster_centers_  # Clusters coordinate
    labels_unique = np.unique(labels)  # Simplifies labels notation
    nb_clusters_ = len(labels_unique)  # Number of clusters. Could remove the line

    print("The number of estimated clusters : %d" % nb_clusters_)
    # print cluster_centers

    markerArray = MarkerArray()
    for i in range(0, nb_clusters_):

        m = Marker()

        # Set the frame id and timestanp
        m.header.frame_id = "robot"
        m.header.stamp = rospy.get_rostime()  # rospy.Time.now()
        # Marker namespace.
        m.ns = "sonar"
        # Marker ID.
        m.id = i  # All of the cluster from one scan have the same id
        # Marker type
        m.type = Marker.POINTS
        # Action.
        m.action = Marker.ADD

        # Set pose of the marker
        # No need for orientation as we use points!
        m.pose.position.x = 0
        m.pose.position.y = 0
        m.pose.position.z = 0

        # Alpha must be set or marker will be invisible.
        m.color.a = 1.0
        # Scale of the Marker.
        m.scale.x = 1.0  # 1m
        m.scale.y = 1.0  # 1m
        m.scale.z = 0.0
        # Lifetime of point.
        m.lifetime.secs = 0

        # Points list holds a single point which corresponds
        # to the cluster location.
        p = Point32()
        p.x = cluster_centers[i][0]
        p.y = cluster_centers[i][1]
        p.z = 0

        m.points = [p]  # redefine the value of the parameter points of the marker
        label_colour(m, i)

        markerArray.markers.append(m)

    # We wait for the markerArray to have a subscriber and then publish it
    pub.publish(markerArray)


def label_colour(m, label):
    """ Provides a unique color for each cluster

    """
    if label == 0:
        # Red.
        m.color.r = 1.0
        m.color.g = 0
        m.color.b = 0
    if label == 1:
        # Green.
        m.color.r = 0
        m.color.g = 1.0
        m.color.b = 0
    if label == 2:
        # Blue.
        m.color.r = 0
        m.color.g = 0
        m.color.b = 1.0
    if label == 3:
        # Yellow.
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0
    if label == 4:
        # Cyan.
        m.color.r = 0
        m.color.g = 1.0
        m.color.b = 1.0
    if label == 5:
        # Cyan.
        m.color.r = 0
        m.color.g = 0.5
        m.color.b = 1.0
    if label == -1:
        # White. Outliers.
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0


if __name__ == '__main__':
    rospy.init_node("Mean_Shift")
    sub = rospy.Subscriber("filtered_scan", PointCloud,
                           populate, queue_size=1)
    pub = rospy.Publisher("visualization_marker_array", MarkerArray,
                          queue_size=10)
    rospy.spin()
