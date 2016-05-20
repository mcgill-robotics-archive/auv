#!/usr/bin/env python

import rospy
import beam_filter
from sensor_msgs.msg import PointCloud


def preprocess(scan):
    scan_slice = beam_filter()

    scan_slice.header.frame_id = "robot"
    scan_slice.header.stamp = rospy.get_rostime()



if __name__ == "__main__":
    rospy.init_node("slice_scan_stitcher")
    scan_sub = rospy.Subscriber("filtered_slice_scan", PointCloud,
                                 queue_size=1)
    scan_pub = rospy.Publisher("filtered_scan", PointCloud,
                                 queue_size=1)

    rospy.spin()
