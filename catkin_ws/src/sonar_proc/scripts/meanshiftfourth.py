#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""MeanShift Clustering Test.

Inspiration from plot_mean_shift.py example from SciKit Learn:

http://scikit-learn.org/stable/auto_examples/cluster/plot_mean_shift.html#example-cluster-plot-mean-shift-py

Publishes:
    visualization_marker_array:

Subscribes:
    /filtered_scan: PointCloud message containing filtered sonar scan

Currently in the process of increasing the scan filtering level!

"""

import rospy
import numpy as np
from std_msgs.msg import Int32, Float32
from sklearn.cluster import MeanShift, estimate_bandwidth
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

__author__ = "Dihia Idrici, Jana Pavlasek"


X = None
markerArray = MarkerArray()

""" meanshiftfourth.py studies the behaviour of the meanshift clustering
    algorithm given only the x and y position of the filtered data.


    Once the PointCloud data from the sonar is filtered, keeping only the
    relevent point there should be no use of the intensity parameter
    to determine clusters.

    After testing, however, we note that more filtering is required,
    as clusters with only few data are taken into account as well. Even though
    they seem to represent nothing.
"""

"""
def sample_point(sample):

    global n_sample
    n_sample = int(sample.data)
    print ("n_sample is %d") % n_sample
"""

def populate(data):
    """The algorithm starts by making a copy of the original fitered data
    set from the topic "full_filtered_scan"
    and freezing the original points. The copied points
    are shifted against the original frozen points.
    """

    # Define array named pts which we fill with the data points
    # from a given sonar scan
    pts = []

    # zip allows us to iterate over two lists in parallel
    for point in data.points:
        # Fill points array with x, y
        # we ommit z as it is always zero (2D representation)
        pts.append([point.x, point.y])

    # Transform data into usuable Numpy arrays.
    global X
    X = np.array(pts)  # Matrix with two column

    # RESET markerArray

    cluster()


def cluster():

    """Clustering with MeanShift"""

    # print ("n_sample second is %d") %n_sample
    # i = n_sample
    # print ("i is %d") % i
    # Bandwidth has to be estimated
    bandwidth = estimate_bandwidth(X, quantile=0.2)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
    ms.fit(X)
    labels = ms.labels_
    cluster_centers = ms.cluster_centers_  # Clusters coordinate
    labels_unique = np.unique(labels)  # Simplifies labels notation
    nb_clusters_ = len(labels_unique)  # Number of clusters. Could remove the line

    print("The number of estimated clusters : %d" % nb_clusters_)
    # print cluster_centers

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
        m.scale.x = 0.25  
        m.scale.y = 0.25 
        m.scale.z = 0.25
        # Lifetime of point.
        m.lifetime.secs = 5

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
    """Provides a unique color for each cluster"""
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
    if label == 6: 
        # Cyan.
        m.color.r = 0
        m.color.g = 0.3
        m.color.b = 0.7
    if label == -1:
        # White. Outliers.
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0


if __name__ == '__main__':
    rospy.init_node("Mean_Shift")
    sub = rospy.Subscriber("filtered_scan", PointCloud,
                           populate, queue_size=1)
    # sample_sub = rospy.Subscriber("n_sample", Int32, sample_point, queue_size=1)
    pub = rospy.Publisher("visualization_marker_array", MarkerArray,
                          queue_size=10)
    rospy.spin()
