#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Task Point

Publishes a point dependant upon current visible clusters and the current task

Two classes of task are considered:
1. direct approach
    look for the cluster centroid closest to the x-axis (i.e. y = 0)
2. maneuver
    look for two cluster centroids that are a fixed distance apart and publish
    the point between those two centroids

subscribes:
    /task
        auv_msg/TaskStatus
    /sonar_proc/cluster_data
        sonar_proc/ClusterArray

publishes:
    sonar_proc/task_point
        geometry_msg/Point

"""
import math
import itertools

import rospy
from geometry_msgs.msg import Point
from auv_msgs.msg import TaskStatus
from sonar_proc.msg import ClusterArray

state = None
DIRECT_APPROACH = [TaskStatus.OCTAGON, TaskStatus.SQUARE, TaskStatus.TORPEDO]
GATE_APPROACH = [TaskStatus.GATE, TaskStatus.MANEUVER]
BUOY_APPROACH = [TaskStatus.BUOYS]

# TODO: this is a completely made-up value
# needs adjusting to the real thing
# this is the distance between the "manuever centroids"

MANEUVER_LENGTH = rospy.get_param("~manuever_length", default=5)
BUOY_SIZE = rospy.get_param("~buoy_size", default=2)


def update_state(data):
    global state
    state = data


def get_distance(x, y):
    return abs(x - y)


def centroid_distance(c1, c2):
    return math.sqrt(abs(c1.centroid.x - c2.centroid.x)**2 +
                     abs(c1.centroid.y - c2.centroid.y)**2)


def get_size_error(test_size, perf_size):
    return abs(perf_size - test_size)


def direct_approach_point(data):
    # search for cluster closest to y axis
    significant_cluster = data.clusters[0]
    min_distance = get_distance(data.clusters[0].centroid.y, 0)
    for cluster in data.clusters:
        distance = get_distance(cluster.centroid.y, 0)
        if distance < min_distance:
            significant_cluster = cluster
            min_distance = distance

    return Point(
        significant_cluster.centroid.x,
        significant_cluster.centroid.y,
        0
    )


def gate_approach_point(data):
    # Creates every combination of two clusters
    cluster_combos = itertools.combinations(data.clusters, 2)

    # Search for the pair of clusters whose distances are closest to the
    # distance between the gate posts
    significant_combo = cluster_combos[0]
    error = centroid_distance(significant_combo[0], significant_combo[1])
    min_error = get_distance(error, MANEUVER_LENGTH)
    for combo in cluster_combos:
        error = get_distance(combo[0], combo[1])
        if error < min_error:
            significant_combo = combo
            min_error = error

    # Return the point in between the gate posts
    centroids = (
        significant_combo[0].centroid,
        significant_combo[1].centroid
    )
    return Point(
        (centroids[0].x + centroids[1].x) / 2,
        (centroids[0].y + centroids[1].y) / 2,
        0
    )


def task_point(data):
    if not state:
        return

    point = None
    if state.task in DIRECT_APPROACH:
        point = direct_approach_point()
    elif state.task in GATE_APPROACH:
        point = gate_approach_point()
    else:
        return

    rospy.loginfo("Sonar found task point at: {%.2f, %.2f}" %
                  (point.x, point.y))
    pub_goal.publish(point)


if __name__ == '__main__':
    rospy.init_node("task_point")
    rospy.Subscriber("sonar_proc/cluster_data", ClusterArray, task_point,
                     queue_size=10)
    rospy.Subscriber("/task", TaskStatus, update_state, queue_size=10)

    pub_goal = rospy.Publisher("sonar_proc/task_point", Point, queue_size=10)
    rospy.spin()
