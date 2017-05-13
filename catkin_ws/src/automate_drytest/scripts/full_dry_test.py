#!/usr/bin/env python
import drytest_thruster
import sensors_dry_test
import os
import rospy

if __name__ == "__main__":
	rospy.init_node("drytest_node")

	#start camera feeds
	print "Starting front camera feed...\n"
	os.system("rosrun image_view image_view image:=/camera_front/image_raw &")
	print "Front camera feed started.\n"
	
	print "Starting down camera feed...\n"
	os.system("rosrun image_view image_view image:=/camera_down/image_raw &")
	print "Down camera feed started.\n"
	
	#check for depth sensor
	sensors_dry_test.check_depth_sensor()

	#TODO check for more sensors (imu & sonar)

	#check thrusters
	drytest_thruster.run_test()