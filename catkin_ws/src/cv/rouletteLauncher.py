#!/usr/bin/env python
import rospy
from cv.detectors import roulette_detector

if __name__ == '__main__':
    rospy.init_node('Roulette',anonymous=True)
    bla = roulette_detector.RouletteDetector()
    #serv.start()

    rospy.spin()

