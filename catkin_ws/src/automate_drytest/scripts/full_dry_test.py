#!/usr/bin/env python
import drytest_thruster
import sensors_dry_test
import rospy

if __name__ == "__main__":
    rospy.init_node("drytest_node")

    # Check for depth sensor
    sensors_dry_test.check_depth_sensor()

    # TODO check for more sensors (imu & sonar)

    # Check thrusters
    drytest_thruster.run_test()
