#!/usr/bin/env python

import rospy

import thrusters_test
import sensors_test

if __name__ == "__main__":
    rospy.init_node("drytest_node")

    # Check for depth sensor
    sensors_test.check_depth_sensor()

    # TODO check for more sensors (imu & sonar)

    # Check thrusters
    thrusters_test.run_test()
