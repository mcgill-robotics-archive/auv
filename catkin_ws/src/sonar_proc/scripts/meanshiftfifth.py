#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""MeanShift Clustering Test.

    Inspiration from plot_mean_shift.py example from SciKit Learn:

http://scikit-learn.org/stable/auto_examples/cluster/plot_mean_shift.html#example-cluster-plot-mean-shift-py


    meanshiftfifth.py studies the behaviour of the meanshift clustering
    algorithm feeding in only the x and y position of the filtered data 
    as parameters

    The reasoning behind this decision is that once the PointCloud data 
    from the sonar is filtered, keeping only the relevent points, there should 
    be no use of the intensity parameter to determine clusters.

    Publishes: 
    /visualization_marker_array: MarkerArray message containing a given number 
                                 of clusters 
                                 
    Subscribe:
    /filtered_scan: PointCloud message containing filtered sonar scan

"""


import rospy
import numpy as np
from Clustering import Clustering
from std_msgs.msg import Int32, Float32
from sklearn.cluster import MeanShift, estimate_bandwidth
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray

__author__ = "Dihia Idrici"

MSC = None

def cluster(data):
    """Clustering with MeanShift"""

    MSC = Clustering()  # Create object MSC
    X = MSC.populate(data)  # Placing the points coordinate of the point cloud in a readable form/ numpy array

    # Bandwidth has to be estimated
    bandwidth = estimate_bandwidth(X, quantile=0.2)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True, cluster_all=False)
    ms.fit(X)
    labels = ms.labels_
    cluster_centers = ms.cluster_centers_  # Clusters coordinate
    labels_unique = np.unique(labels)  # Simplifies labels notation
    # nb_clusters_ = len(labels_unique)  # Number of clusters. Could remove the line
    """I removed nb_clusters = np.unique(labels) because I noticed that sometimes 
    the len(labels_unique) was greater by +1 that of the real number of cluster_centers
    """
    nb_clusters_ = len(cluster_centers) 
    print("The number of estimated clusters : %d" % nb_clusters_)
    print cluster_centers

    markerA = MSC.CreateMarkerArray(cluster_centers, nb_clusters_)
    pub.publish(markerA)



if __name__ == '__main__':
    rospy.init_node("Mean_Shift_Fifth")
    sub = rospy.Subscriber("filtered_scan", PointCloud, cluster, queue_size=1)
    # sample_sub = rospy.Subscriber("n_sample", Int32, cluster, queue_size=1)
    pub = rospy.Publisher("visualization_marker_array", MarkerArray,
                          queue_size=10)

    rospy.spin()

