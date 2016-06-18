#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from itertools import cycle
from sklearn.cluster import MeanShift, estimate_bandwidth
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
from sklearn.preprocessing import StandardScaler

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


    for point in data.points: # Range [0, 255]
        # Fill points array with x, y and intensity value
        # we ommit z as it is always zero (2D representation)
        pts.append([point.x, point.y])

    # Transform data into usuable Numpy arrays.
    global X
    # X_initial = np.array(pts)
    # X = StandardScaler().fit_transform(X_initial)
    X = np.array(pts)

    cluster()


def cluster():

    # Compute clustering with MeaShift
    # Bandwidth detection
    bandwidth = estimate_bandwidth(X, quantile=0.3, n_samples=None)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
    ms.fit(X)
    labels = ms.labels_
    cluster_centers = ms.cluster_centers_
    labels_unique = np.unique(labels)
    n_clusters_ = len(labels_unique)

    # print X
    # print cluster_centers
    print("number of estimated clusters : %d" % n_clusters_)

    # Plot result

    plt.figure(1)
    plt.clf()

    colors = cycle('bgrcmykbgrcmykbgrcmykbgrcmyk')
    for k, col in zip(range(n_clusters_), colors):
        my_members = labels == k
        cluster_center = cluster_centers[k]
        # This plots the set of data point
        plt.plot(X[my_members, 0], X[my_members, 1], col + '.')
        plt.plot(cluster_center[0], cluster_center[1], 'o',
                 markerfacecolor=col, markeredgecolor='k', markersize=14)
    plt.title('Estimated number of clusters: %d' % n_clusters_)
    plt.show()


if __name__ == '__main__':
    rospy.init_node("Mean_Shift")
    sub = rospy.Subscriber("/filtered_scan", PointCloud,
                           populate, queue_size=1)
    pub = rospy.Publisher("/cluster_markers", Marker, queue_size=100)
    rospy.spin()
