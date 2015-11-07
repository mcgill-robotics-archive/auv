#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Hydrophones signal analysis."""

import rospy
from hydrophones import Mic
from auv_msgs.msg import Signals, TDOAs
from hydrophones.gccphat import estimate
from hydrophones.multilateration import solve
from hydrophones.freq import get_frequency_energy

FS = 972972.97297

mics = [
    Mic((0.00, 0.00, 0.00)),
    Mic((-0.013, 0.00, 0.00)),
    Mic((-0.013, -0.013, 0.00)),
    Mic((0.00, -0.013, 0.00))
]


def analyze(msg):
    print(get_frequency_energy(msg.quadrant_1, FS, 28000))
    tdoa = [
        estimate(msg.quadrant_2, msg.quadrant_1, FS),
        estimate(msg.quadrant_2, msg.quadrant_2, FS),
        estimate(msg.quadrant_2, msg.quadrant_3, FS),
        estimate(msg.quadrant_2, msg.quadrant_4, FS)
    ]
    tdoas = TDOAs()
    tdoas.port_bow_starboard_bow = tdoa[0]
    tdoas.port_bow_starboard_stern = tdoa[3]
    tdoas.port_bow_port_stern = tdoa[2]
    pub.publish(tdoas)
    for dt in tdoa:
        print(dt)
    x, y = solve(mics, tdoa, 1500.0)
    print(x, y)
    print("---")


if __name__ == "__main__":
    global pub
    rospy.init_node("tdoa")
    rospy.Subscriber("/nucleo/signals", Signals, analyze, queue_size=10)
    pub = rospy.Publisher("hydrophones", TDOAs, queue_size=100)
    rospy.spin()
