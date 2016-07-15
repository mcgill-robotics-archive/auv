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

import rospy
from geometry_msgs.msg import Point
from auv_msgs.msg import TaskStatus
from sonar_proc.msg import ClusterArray

state = None
# TODO: refactor this list into a config file
DIRECT_APPROACH = [TaskStatus.GATE, TaskStatus.BUOYS, TaskStatus.OCTAGON,
                   TaskStatus.SQUARE, TaskStatus.TORPEDO]
# TODO: this is a completely made-up value 
# needs adjusting to the real thing
# this is the distance between the "manuever centroids"
MANEUVER_DISTANCE = rospy.get_param("sonar_proc/manuever_distance", default=5)

def update_state(data):
    global state
    state = data


def get_distance(x, y):
    return abs(x - y)


def centroid_distance(c1, c2):
    return math.sqrt(abs(c1.centroid.x - c2.centroid.x)**2 + abs(c1.centroid.y - c2.centroid.y)**2)


def task_point(data):
    if not state:
        return

    yaxis_index = None
    distance = None
    for i, cluster in enumerate(data.clusters):
        if not yaxis_index or get_distance(cluster.centroid.y, 0) < yaxis_index[1]:
            yaxis_index = (i, get_distance(cluster.centroid.y, 0))

        # TODO: Fix double calculation
        # Right now this loop is calculating the distances between each two centroids twice
        # We should store that info so we can cut the calcs in half
        for c in data.clusters[:i] + data.clusters[i:]:
            if not distance or get_distance(centroid_distance(cluster, c), MANEUVER_DISTANCE) < distance[1]:
                distance = (i, get_distance(centroid_distance(cluster, c), MANEUVER_DISTANCE))

    significant_cluster = None
    if state.task == TaskStatus.MANEUVER:
        significant_cluster = data.clusters[distance[0]]
    elif state.task in DIRECT_APPROACH:
        significant_cluster = data.clusters[yaxis_index[0]]
    else:
        return

    point = Point(significant_cluster.centroid.x, significant_cluster.centroid.y, 0)

    rospy.loginfo(point)
    pub_goal.publish(point)


if __name__ == '__main__':
    rospy.init_node("task_point")
    rospy.Subscriber("sonar_proc/cluster_data", ClusterArray, task_point, queue_size=1)
    rospy.Subscriber("/task", TaskStatus, update_state, queue_size=1)
    pub_goal = rospy.Publisher("sonar_proc/task_point", Point, queue_size=1)

    rospy.spin()
