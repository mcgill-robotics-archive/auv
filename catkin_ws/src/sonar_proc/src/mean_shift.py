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
from sklearn.cluster import MeanShift, estimate_bandwidth

from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray

from sonar_proc.msg import Cluster, ClusterArray
from clustering import Clustering

__author__ = "Justin Bell, Dihia Idrici"


def cluster(data):
    """Clustering with MeanShift"""
    if not data.points:
        rospy.loginfo("Clustering was handed an empty set")
        return

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

    rospy.loginfo("number of clusters : {}".format(number_of_clusters))
    rospy.loginfo("average intensities : {}".format(average_intensities))
    rospy.loginfo("sizes : {}".format(sizes))
    rospy.loginfo("cluster centers : {}".format(cluster_centers))

    cluster_array = ClusterArray()
    cluster_array.header.stamp = rospy.get_rostime()
    cluster_array.header.frame_id = data.header.frame_id

    clusters = zip(cluster_centers, average_intensities, sizes)
    for center, average_intensity, size in clusters:
        cluster = Cluster()
        cluster.centroid.x, cluster.centroid.y = (center[0], center[1])
        cluster.average_intensity = average_intensity
        cluster.size = size
        cluster_array.clusters.append(cluster)

    marker_pub.publish(markers)
    pub_clusters_data.publish(cluster_array)


if __name__ == '__main__':
    rospy.init_node("mean_shift")
    sub = rospy.Subscriber("/filtered_scan/pcl_filtered", PointCloud, cluster, queue_size=1)
    marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray,
                                 queue_size=10)
    pub_clusters_data = rospy.Publisher("sonar_proc/cluster_data", ClusterArray, queue_size=1)

    rospy.spin()
