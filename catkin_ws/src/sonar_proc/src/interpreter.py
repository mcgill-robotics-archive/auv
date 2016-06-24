#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Interpreter

Takes in an array of clusters and publishes a Point
which represents the centroid of the most 'significant' cluster

We consider closeness to the x-axis, average intensity,
and closeness to the mean of cluster sizes for determining
the most 'significant' cluster

subscribes:
    sonar_proc/cluster_data
        sonar_proc/ClusterArray

publishes:
    sonar_proc/goal
        geometry_msg/Point

"""

import rospy
from geometry_msgs.msg import Point

from sonar_proc.msg import ClusterArray

# parameter weightings
DISTANCE=0.5
SIZE=0.25
INTENSITY=0.25

def _normalize_helper(max_value, min_value):
    def helper(x):
        return (x - min_value) / (max_value - min_value)
    return helper

def _normalize(values):
    max_value = max(values)
    min_value = min(values)

    # prevent a division by zero
    if max_value == min_value:
        return [1.0]*len(values)

    helper = _normalize_helper(max_value, min_value)
    return map(lambda x: helper(x), values)

def distance(x, y):
    return abs(x - y)

def callback(data):
    xaxis_distances, sizes, intensities = [], [], []
    for cluster in data.clusters:
        xaxis_distances.append(distance(cluster.centroid.x, 0))
        sizes.append(cluster.size)
        intensities.append(cluster.average_intensity)

    distance_scores = _normalize(xaxis_distances)

    # we want to compare clusters based on distance of cluster size to
    # average cluster size
    mean_size = sum(sizes) / len(sizes)
    size_scores = _normalize([distance(x, mean_size) for x in sizes])

    intensity_scores = _normalize(intensities)

    # apply the weighting
    scores = zip(distance_scores, size_scores, intensity_scores)
    for score in scores:
        score = [score[0]*DISTANCE + score[1]*SIZE + score[2]*INTENSITY]

    # we want the index of the largest score
    i = reduce(lambda x, y: x if x[1] > y[1] else y, enumerate(score))[0]

    significant_cluster = data.clusters[i]
    point = Point(significant_cluster.centroid.x, significant_cluster.centroid.y, 0)

    pub_goal.publish(point)

if __name__ == '__main__':
    rospy.init_node("interpreter")
    sub = rospy.Subscriber("sonar_proc/cluster_data", ClusterArray, callback, queue_size=1)
    pub_goal = rospy.Publisher("sonar_proc/goal", Point, queue_size=1)

    rospy.spin()
