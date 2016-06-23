#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""MeanShift Clustering Test.

    Inspiration from plot_mean_shift.py example from SciKit Learn:

http://scikit-learn.org/stable/auto_examples/cluster/plot_mean_shift.html#example-cluster-plot-mean-shift-py

    Meanshift code on github:

https://github.com/scikit-learn/scikit-learn/blob/master/sklearn/cluster/mean_shift_.py

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
import numpy
from sklearn.cluster import MeanShift, estimate_bandwidth

from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray


from clustering import Clustering

__author__ = "Dihia Idrici"

def cluster(data):
    """Clustering with MeanShift"""

    clustering = Clustering(data)

    # Creates a meanshift clusterer and fits the point cloud data
    bandwidth = estimate_bandwidth(clustering.numpy_points, quantile=0.2)
    meanshift = MeanShift(bandwidth, bin_seeding=True, cluster_all=False)
    meanshift.fit(clustering.numpy_points)

    labels = meanshift.labels_
    number_of_clusters = len(meanshift.cluster_centers_)

    # collect cluster features
    cluster_centers = meanshift.cluster_centers_
    average_intensities = clustering.get_average_intensities(labels, number_of_clusters)
    sizes = clustering.get_sizes(labels, number_of_clusters)

    # get markers for the cluster centers
    markers = clustering.make_markers(cluster_centers)

    print("number of clusters : %d" % number_of_clusters)
    print("average intensities : " + str(average_intensities))
    print("sizes : " + str(size))
    print("cluster centers : " + str(cluster_centers))

    marker_pub.publish(markers)
    # pub_clusters_data.publish(clusters_information)


if __name__ == '__main__':
    rospy.init_node("mean_shift")
    sub = rospy.Subscriber("filtered_scan", PointCloud, cluster, queue_size=1)
    marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray,
                          queue_size=10)
    # pub_clusters_data = rospy.Publisher("cluster_data", Int32, queue_size=1)

    rospy.spin()

