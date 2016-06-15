#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray, Marker


class Clustering(object):

    # def __init__(self, n_sample=sample_sub, data=sub):
    def __init__(self):
    	# self.n_sample = n_sample
        # self.samplesize = None
    	# self.data = sub
        self.pts = []
	self.X = None
        self.markerArray = MarkerArray()
        self.m = Marker()
        self.p = Point32()


    def populate(self, data):
	#for point in self.data.points
	for point in data.points:
	    self.pts.append([point.x, point.y])

	self.X = np.array(self.pts)
	# print self.X
	return self.X

    def CreateMarkerArray(self, cluster_centers, nb_clusters_):

        for i in range(0, nb_clusters_):

            # Set the frame id and timestanp
            self.m.header.frame_id = "robot"
            self.m.header.stamp = rospy.get_rostime()  # rospy.Time.now()
            # Marker namespace.
            self.m.ns = "sonar"
            # Marker ID.
            self.m.id = i  # All of the cluster from one scan have the same id
            # Marker type
            self.m.type = Marker.POINTS
            # Action.
            self.m.action = Marker.ADD

            # Set pose of the marker
            # No need for orientation as we use points!
            self.m.pose.position.x = 0
            self.m.pose.position.y = 0
            self.m.pose.position.z = 0

            # Alpha must be set or marker will be invisible.
            self.m.color.a = 1.0
            # Scale of the Marker.
            self.m.scale.x = 0.25  
            self.m.scale.y = 0.25  
            self.m.scale.z = 0.25
            # Lifetime of point.
            self.m.lifetime.secs = 0

            # Points list holds a single point which corresponds
            # to the cluster location.
        
            self.p.x = cluster_centers[i][0]
            self.p.y = cluster_centers[i][1]
            self.p.z = 0

            self.m.points = [self.p]
            label_colour(self.m, i)

            self.markerArray.markers.append(self.m)

        # We wait for the markerArray to have a subscriber and then publish it
        return self.markerArray


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
    if label == 5:
    #
        m.color.r = 0
        m.color.g = 0.5
        m.color.b = 1.0
    if label == 6:
    #
        m.color.r = 0
        m.color.g = 0.3
        m.color.b = 0.7
    if label == -1:
    # White. Outliers.
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0

