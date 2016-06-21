#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
import numpy as np
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


__author__ = "Dihia Idrici, Jana Pavlasek" 

"""mean_shift.py uses this class, to generate a new MarkerArray
   containing markers (or clusters) for each input from /filtered_scan topic 

   Depending on the angle range we will select for our sonar scan, the 
   markers lifetime (self.m.lifetime.sec) will have to be adjusted
"""


class Clustering(object):

    def __init__(self):
        self.pts = []
	self.X = None
        self.markerArray = False
        self.p = None


    def populate(self, data):
        """x and y position of each point within the filtered 
        PointCloud placed into a feature array for clustering"""
	for point in data.points:
	    self.pts.append([point.x, point.y])

        # Transforms data into usuable Numpy Array
	self.X = np.array(self.pts)
        return self.X


    def CreateMarkerArray(self, cluster_centers, nb_clusters_):

        # print cluster_centers and nb_clusters_/ 
        # This is meant to verify the information is being sent 
        # properly to the class
        # print cluster_centers
        # print nb_clusters_
        if not self.markerArray:
            self.markerArray = MarkerArray()

            for i in range(0, nb_clusters_):
                # each m is a cluster
                self.m = Marker()

                # Set the frame id and timestanp
                self.m.header.frame_id = "robot"
                self.m.header.stamp = rospy.get_rostime()  # rospy.Time.now()
                # Marker namespace.
                self.m.ns = "sonar"
                # Marker ID 
                self.m.id = i 
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
                self.m.scale.x = 0.20  
                self.m.scale.y = 0.20  
                self.m.scale.z = 0.20
                # Lifetime of point.
                self.m.lifetime.secs = 6

                # Points list holds a single point which corresponds
                # to the cluster location.
                self.p = Point32()
                self.p.x = cluster_centers[i][0]
                self.p.y = cluster_centers[i][1]
                self.p.z = 0

                self.m.points = [self.p]
                label_colour(self.m, i)

                self.markerArray.markers.append(self.m)
            
            # Return to meanshiftfifth.py cluster() function
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
        m.color.r = 0
        m.color.g = 0.5
        m.color.b = 1.0
    if label == 6:
        m.color.r = 0
        m.color.g = 0.3
        m.color.b = 0.7
    if label == 7:
        m.color.r = 0.3
        m.color.g = 0.7
    if label == -1:
    # White. Outliers.
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0

