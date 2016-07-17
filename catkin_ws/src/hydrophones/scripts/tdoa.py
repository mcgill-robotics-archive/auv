#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Hydrophones signal analysis."""

import rospy
import math
from hydrophones import Mic
from auv_msgs.msg import Signals
from std_msgs.msg import Float64
from hydrophones.gccphat import estimate
from hydrophones.multilateration import solve
from hydrophones.freq import get_frequency_energy

FS = 1028571.4286

def analyze(msg):
    tdx = estimate(msg.quadrant_1[:-6], msg.quadrant_2[6:], FS)
    tdy = estimate(msg.quadrant_1[:-10], msg.quadrant_4[10:], FS)
    yaw = Float64()
    yaw.data = math.atan2(tdx, tdy)
    pub.publish(yaw)

if __name__ == "__main__":
    rospy.init_node("hydrophones")
    pub = rospy.Publisher("~heading", Float64, queue_size=1)
    rospy.Subscriber("/nucleo/signals", Signals, analyze, queue_size=10)
    rospy.spin()
