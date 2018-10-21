#!/usr/bin/env python
import rospy
from cv.detectors import roulette_detector
from cv.detectors import lane_detector
from cv.detectors import servo_practice
from controls.controllers import visual_servo

if __name__ == '__main__':
    rospy.init_node('ServoPractice',anonymous=True)
    serv = visual_servo.DownVisualServoController()
    bla = lane_detector.LaneDetector()
    serv.start()

    rospy.spin()

