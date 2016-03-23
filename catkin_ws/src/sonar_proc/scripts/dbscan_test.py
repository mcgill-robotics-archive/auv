#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""DBSCAN Clustering Test.

Finds core samples of high density and expands clusters from them. Some of
this is from the plot_dbscan.py example from SciKit Learn:

http://scikit-learn.org/stable/auto_examples/cluster/plot_dbscan.html#example-cluster-plot-dbscan-py

Publishes:
    /cluster_markers: Marker message with different coloured markers
                      corresponding to each cluster (only 4 clusters are
                      supported right now, this is hard coded.)
Subcribes:
    /full_scan: PointCloud message containing complete sonar scan.
"""

import rospy
import numpy as np
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
from sklearn.preprocessing import StandardScaler

__author__ = "Jana Pavlasek"

X = None
X_initial = None


def populate(data):
    """Callback that turns PointCloud data into feature array for
    clustering."""
    pts = []
    # Add points to the array.
    for point in data.points:
        pts.append([point.x, point.y])

    # Add intensities to each point array. Eliminate points under a certain
    # threshold (Note: this will not be needed once preprocessing is
    # finished).
    i = 0
    for val in data.channels[0].values:
        intensity = val / 100.0  # Change intensity from [0, 100] to [0, 1]
        if intensity > 0.0:
            pts[i].append(intensity)
        else:
            pts.pop(i)
            i -= 1
        i += 1

    # Transform data into usable NumPy arrays.
    global X, X_initial
    X_initial = np.array(pts)
    X = StandardScaler().fit_transform(X_initial)

    cluster()


def cluster():
    """Clusters the data then publishes markers for RVIZ visualization."""
    # Compute DBSCAN. Default: eps=0.3, min_samples=10
    db = DBSCAN(eps=0.3, min_samples=10).fit(X)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

    print('Estimated number of clusters: %d' % n_clusters_)

    for label, i in zip(labels, range(0, len(labels))):
        m = Marker()
        # Set default values.
        construct_marker(m)

        # Marker ID.
        m.id = i

        # Position is that of the point at the same index.
        m.pose.position.x = X_initial[i][0]
        m.pose.position.y = X_initial[i][1]
        m.pose.position.z = 0

        # Points list holds a single point.
        p = Point()
        p.x = X_initial[i][0]
        p.y = X_initial[i][1]
        p.z = 0
        m.points = [p]

        # Colours.
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
        if label == -1:
            # White. Outliers.
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
        pub.publish(m)


def construct_marker(m):
    """Contructs Marker message with defaults.

    Args:
        m = Marker message.
    """
    # Header
    m.header.frame_id = "robot"  # This might be broken.
    m.header.stamp = rospy.Time.now()
    # Namespace.
    m.ns = "sonar"
    # Action.
    m.action = Marker.ADD
    # Type of marker.
    m.type = Marker.POINTS
    # Alpha must be set or marker will be invisible.
    m.color.a = 1.0
    # Scale.
    m.scale.x = 0.05
    m.scale.y = 0.05
    m.scale.z = 0
    # Lifetime of point.
    m.lifetime.secs = 5


if __name__ == '__main__':
    rospy.init_node("test_cluster")
    sub = rospy.Subscriber("/filtered_scan", PointCloud,
                           populate, queue_size=1)
    pub = rospy.Publisher("/cluster_markers", Marker, queue_size=1)
    rospy.spin()
