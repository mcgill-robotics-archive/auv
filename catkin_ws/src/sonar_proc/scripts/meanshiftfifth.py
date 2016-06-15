#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from Clustering import * 
from std_msgs.msg import Int32, Float32
from sklearn.cluster import MeanShift, estimate_bandwidth
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker



def cluster(data):
    """Clustering with MeanShift"""

    MSC = Clustering()
    X = MSC.populate(data)

    # Bandwidth has to be estimated
    bandwidth = estimate_bandwidth(X, quantile=0.2)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True, cluster_all=False)
    ms.fit(X)
    labels = ms.labels_
    cluster_centers = ms.cluster_centers_  # Clusters coordinate
    labels_unique = np.unique(labels)  # Simplifies labels notation
    nb_clusters_ = len(labels_unique)  # Number of clusters. Could remove the line

    print("The number of estimated clusters : %d" % nb_clusters_)



    markerA = MSC.CreateMarkerArray(cluster_centers, nb_clusters_)
    pub.publish(markerA)


"""
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
        m.scale.x = 0.25  # 1m
        m.scale.y = 0.25  # 1m
        m.scale.z = 0.25
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
    # Provides a unique color for each cluster


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

"""

if __name__ == '__main__':
    rospy.init_node("Mean_Shift")
    sub = rospy.Subscriber("filtered_scan", PointCloud, cluster, queue_size=1)
    # sample_sub = rospy.Subscriber("n_sample", Int32, cluster, queue_size=1)
    pub = rospy.Publisher("visualization_marker_array", MarkerArray,
                          queue_size=10)

    rospy.spin()

