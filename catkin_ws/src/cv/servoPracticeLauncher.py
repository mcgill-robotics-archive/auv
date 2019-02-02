#!/usr/bin/env python
import rospy
from cv.detectors import roulette_detector
from cv.detectors import lane_detector
from cv.detectors import servo_practice
from controls.controllers import visual_servo
from controls.maintainers import yaw_maintainer

if __name__ == '__main__':
    rospy.init_node('ServoPractice',anonymous=True)
    serv = visual_servo.DownVisualServoController()
    yaw = yaw_maintainer.YawMaintainer()
    yaw.start()
    bla = servo_practice.ServoPracticeD()
    serv.start()

    rospy.spin()

