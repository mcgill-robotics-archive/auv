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
from visualization_msgs.msg import Marker
from std_msgs.msg._ColorRGBA import ColorRGBA
from auv_msgs.msg import TaskPointsArray

state = None
# TODO: refactor this list into a config file

# TODO: This has been commented out for testing purposes.
DIRECT_APPROACH = [TaskStatus.GATE, TaskStatus.BUOYS, TaskStatus.OCTAGON, TaskStatus.SQUARE, TaskStatus.TORPEDO]

# TODO: this is a completely made-up value
# needs adjusting to the real thing
# this is the distance between the "manuever centroids"

# TODO: This has been commented out for testing purposes.
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
    significant_cluster = None

    marker = Marker()
    marker.header.frame_id = "robot"
    marker.header.stamp = rospy.get_rostime()
    marker.type = Marker.POINTS
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.lifetime.secs = 8

    taskArr = TaskPointsArray()
    dummyPoint = Point(0, 0, 0)

    if state.task in DIRECT_APPROACH:
        for i, cluster in enumerate(data.clusters):
            if not yaxis_index or get_distance(cluster.centroid.y, 0) < yaxis_index[1]:
                yaxis_index = (i, get_distance(cluster.centroid.y, 0))
        significant_cluster = data.clusters[yaxis_index[0]]
        point = Point(significant_cluster.centroid.x, significant_cluster.centroid.y, 0)
        marker.points = [point]
        marker.color = ColorRGBA(1, 0, 0, 1)
        taskArr.task = point
        taskArr.pole1 = dummyPoint
        taskArr.pole2 = dummyPoint

    elif state.task == TaskStatus.MANEUVER:
        for i, cluster in enumerate(data.clusters):
            if i == (len(data.clusters)) - 1:
                continue
            for c in data.clusters[(i + 1):]:
                if not distance or get_distance(centroid_distance(cluster, c), MANEUVER_DISTANCE) < distance[1]:
                    distance = (i, get_distance(centroid_distance(cluster, c), MANEUVER_DISTANCE), c)
        clusterOne = data.clusters[distance[0]].centroid
        clusterTwo = distance[2].centroid
        point = Point(((clusterOne.x + clusterTwo.x) / 2), ((clusterOne.y + clusterTwo.y) / 2), 0)
        pointc1 = Point(clusterOne.x, clusterOne.y, 0)
        pointc2 = Point(clusterTwo.x, clusterTwo.y, 0)
        marker.points = [pointc1, point, pointc2]
        marker.color = ColorRGBA(0, 1, 0, 1)
        taskArr.task = point
        taskArr.pole1 = pointc1
        taskArr.pole2 = pointc2

    else:
        return

    rospy.loginfo(point)
    marker_task.publish(marker)
    pub_goal.publish(taskArr)


if __name__ == '__main__':
    rospy.init_node("task_point")
    rospy.Subscriber("sonar_proc/cluster_data", ClusterArray, task_point, queue_size=1)
    # TODO: This has been commented out for testing purposes.
    rospy.Subscriber("/task", TaskStatus, update_state, queue_size=1)
    marker_task = rospy.Publisher("sonar_proc/task_viz", Marker, queue_size=1)
    pub_goal = rospy.Publisher("sonar_proc/task_point", TaskPointsArray, queue_size=1)
    rospy.spin()
