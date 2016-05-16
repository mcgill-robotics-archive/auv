#!/usr/bin/env python

"""MeanShift Clustering Test.

Inspiration from plot_mean_shift.py example from SciKit Learn:

http://scikit-learn.org/stable/auto_examples/cluster/plot_mean_shift.html#example-cluster-plot-mean-shift-py

Publishes:
    /cluster_markers:

Subscribes:
    /full_scan: PointCloud message containing complete sonar scan

"""

import rospy
import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
from sklearn.preprocessing import StandardScaler

__author__ = "Dihia Idrici, Jana Pavlasek"

X = None
X_initial = None


def populate(data):
    """ The algorithm starts by making a copy of the original data set
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
    global X, X_initial
    X_initial = np.array(pts)
    X = StandardScaler().fit_transform(X_initial)

    cluster()


def cluster():

    """Compute clustering with MeanShift"""

    # Bandwidth has to be estimated
    bandwidth = estimate_bandwidth(X, quantile=0.2, n_samples=1000)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
    ms.fit(X)
    labels = ms.labels_
    cluster_centers = ms.cluster_centers_
    labels_unique = np.unique(labels)
    n_clusters_ = len(labels_unique)

    print cluster_centers
    print("The number of estimated clusters : %d" % n_clusters_)

    for label, i in zip(labels, range(0, len(labels_unique))):
        m = Marker()
        # Set default values.
        construct_marker(m)
        # Marker ID.
        m.id = i

        # my_members = labels == label
        # cluster_center = cluster_centers[label]

        # Position of the marker is that of the point at the same index.
        m.pose.position.x = 0
        m.pose.position.y = 0
        m.pose.position.z = 0

        # Points list holds a single point.
        p = Point32()
        p.x = X[i][0]  # instead of X_initial
        p.y = X[i][1]
        p.z = 0
        m.points = [p]

        label_colour(m, label)

        # We wait for the marker to have a subscriber and then publish it
        pub.publish(m)


def construct_marker(m):

    """Contructs Marker message with defaults.

    Args:
        m = Marker message.
    """
    # Header
    m.header.frame_id = "robot"
    m.header.stamp = rospy.Time.now()
    # Namespace.
    m.ns = "sonar"
    # Action.
    m.action = Marker.ADD
    # Type of marker.
    m.type = Marker.POINTS
    # Alpha must be set or marker will be invisible.
    m.color.a = 1.0
    # Scale of the Marker.
    m.scale.x = 1
    m.scale.y = 1
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
    rospy.init_node("Mean_Shift")
    sub = rospy.Subscriber("/filtered_scan", PointCloud,
                           populate, queue_size=1)
    pub = rospy.Publisher("/cluster_markers", Marker, queue_size=100)
    rospy.spin()
