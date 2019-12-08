#!/usr/bin/env python
import rospy
from cv.detectors import roulette_detector
from cv.detectors import lane_detector
from cv.detectors import servo_practice
from controls.controllers import visual_servo

if __name__ == '__main__':
    rospy.init_node('Roulette',anonymous=True)
    serv = visual_servo.DownVisualServoController()
    bla = roulette_detector.RouletteDetector()
    serv.start()

    rospy.spin()

