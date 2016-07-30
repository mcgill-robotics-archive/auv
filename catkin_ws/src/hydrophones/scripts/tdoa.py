#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Hydrophones signal analysis."""

import rospy
import math
import time
from auv_msgs.msg import Signals
from std_msgs.msg import Float64
from hydrophones.gccphat import estimate
from hydrophones.freq import get_frequency


FS = 1028571.4286

last_call = 0

def analyze(msg):
    tdx = estimate(msg.quadrant_1[:-5], msg.quadrant_2[5:], FS)
    tdy = estimate(msg.quadrant_1[:-10], msg.quadrant_4[10:], FS)
    freq = get_frequency(msg.quadrant_1, FS)

    rospy.loginfo("Pingers heard frequency {}".format(freq))

    current_call = time.clock()
    time_delta = current_call - last_call

    if freq == DESIRED_FREQ and time_delta > 0.3:
        last_call = current_call
        yaw = Float64()
        yaw.data = math.atan2(tdx, tdy)
        pub.publish(yaw)

if __name__ == "__main__":
    global pub
    rospy.init_node("hydrophones")

    DESIRED_FREQ = rospy.get_param("hydrophones/desired_freq")

    pub = rospy.Publisher("~heading", Float64, queue_size=1)
    rospy.Subscriber("/nucleo/signals", Signals, analyze, queue_size=10)
    rospy.spin()
