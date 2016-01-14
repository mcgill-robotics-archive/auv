#!/usr/bin/env python

from wait_for_message import *
from std_msgs.msg import Float64

if __name__ == "__main__":
    rospy.init_node("/depth_sensor_test")
    try:
        wait_for_message("/raw_depth", Float64, 3)
    except Exception:
        pass


