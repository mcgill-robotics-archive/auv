#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Manual testing interface for acoustic_servo action."""

import os
import rospy
from std_msgs.msg import Float32

__author__ = "Malcolm Watt"
__location__ = os.path.realpath(os.path.join(os.getcwd(),
                                os.path.dirname(__file__)))


class AcousticTest(object):
    """Test facilitator for the acoustic servo action."""

    def __init__(self):
        """Define proper ranges for values and create publishers."""
        self.ranges = {"imu": range(0, 360), "png": range(0, 8)}
        self.imu_pub = rospy.Publisher("imu_test", Float32, queue_size=10)
        self.png_pub = rospy.Publisher("hyd_test", Float32, queue_size=10)
        self.pubs = {"imu": self.imu_pub, "png": self.png_pub}

    def req_and_pub(self, topic):
        """Request input from the user, validate and publish."""

        # Tell the user what range we expect
        rn = self.ranges[topic]
        print("Input any value from {0} to {1}, and `ctrl-d` to exit"
              .format(rn[0], rn[-1]))

        angle = int(raw_input(topic + "> "))

        if angle in rn:
            self.pubs[topic].publish(Float32(angle))
        else:
            print("Why would you dissobey me?")

if __name__ == "__main__":
    # Display help text using the contents of acoustic_servo_test.txt
    help_text = open(os.path.join(__location__, 'acoustic_servo_test.txt'))
    for line in help_text:
        print(line)

    rospy.init_node("acoustic_servo_test")
    test = AcousticTest()

    while not rospy.is_shutdown():
        for i in range(0, 10):
            test.req_and_pub("imu")  # Request fake imu data and publish
        test.req_and_pub("png")  # Request fake pinger data and publish
