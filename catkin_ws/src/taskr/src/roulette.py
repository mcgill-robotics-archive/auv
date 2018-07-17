#!/usr/bin/env python
import rospy
from cv.detectors import roulette_detector
from controls.maintainers import yaw_maintainer, depth_maintainer
from controls.controllers import visual_servo


class RouletteT(object):

    def __init__(self, data):

        self.preempted = False

        # self.yaw_maintainer = yaw_maintainer.YawMaintainer()
        # self.depth_maintainer = depth_maintainer.DepthMaintainer()
        self.foundCounts = rospy.get_param("/taskr/roulette/foundCounts", 20)
        self.centerStableCounts = rospy.get_param("/taskr/roulette/centerStableCounts", 20)
        self.centerMaxError = rospy.get_param("/taskr/roulette/centerMaxError", 0.4)

    def start(self, server, feedback_msg):
        # TODO: start the CV stuff
        self.roul_detector = roulette_detector.RouletteDetector()
        rospy.loginfo("Roulette Task started.")

        # TODO: leave?
        """
        if not self.yaw_maintainer.is_active():
            self.yaw_maintainer.start()

        if not self.depth_maintainer.is_active():
            self.depth_maintainer.start()
        """
        self.findRoulette()
        if self.preempted:
            return
        self.servoToCenter()
        if self.preempted:
            return

    def stop(self):
        self.preempted = True

        """
        if self.depth_maintainer.is_active():
            self.depth_maintainer.stop()
        if self.yaw_maintainer.is_active():
            self.yaw_maintainer.stop()
        
        """
        # TODO: stop the lane_detector?
        self.roul_detector.stop()


    def findRoulette(self):

        roulFoundCounts = 0
        while roulFoundCounts < self.foundCounts:

            if self.preempted:
                return

            found = self.roul_detector.getRouletteFound()
            if (found):
                roulFoundCounts += 1
            else:
                roulFoundCounts = 0

            rospy.loginfo("{} / {} roulette found.".format(
                roulFoundCounts, self.foundCounts))
            rospy.sleep(0.1)

        rospy.loginfo("Roulette successfully found.")

    def servoToCenter(self):
        rospy.loginfo("Centering over Roulette:")
        self.serv = visual_servo.DownVisualServoController()
        self.serv.start()

        stable_counts = 0
        while stable_counts < self.centerStableCounts:
            rospy.loginfo("Centering: {} / {} stable periods achieved".format(
                stable_counts, self.centerStableCounts))

            if self.preempted:
                return

            err = self.serv.get_error()
            # debug:
            print("Error = {}".format(err))

            if err == (None, None):
                stable_counts = 0

            else:
                err = abs(err[0]) + abs(err[1])
                if err < self.centerMaxError:
                    stable_counts += 1
                else:
                    stable_counts = 0

            rospy.sleep(0.1)

        rospy.loginfo("Done centering action")

