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
from geometry_msgs.msg import Point32
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
    # Add intensities and points to array.
    for point, val in zip(data.points, data.channels[0].values):
        intensity = val  # Range [0, 255].
        pts.append([point.x, point.y, intensity])

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

        cluster_center = cluster_centers[label]
        print cluster_center

        # Marker ID.
        m.id = i

        # Position is that of the point at the same index.
        m.pose.position.x = 0
        m.pose.position.y = 0
        m.pose.position.z = 0

        # Points list holds a single point.
        p = Point32()
        p.x = X_initial[i][0]
        p.y = X_initial[i][1]
        p.z = 0
        m.points = [p]

        label_colour(m, label)

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


def label_colour(m, label):
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


if __name__ == '__main__':
    rospy.init_node("test_cluster")
    sub = rospy.Subscriber("/filtered_scan", PointCloud,
                           populate, queue_size=1)
    pub = rospy.Publisher("/cluster_markers", Marker, queue_size=1)
    rospy.spin()
