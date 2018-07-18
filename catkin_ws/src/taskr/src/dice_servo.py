#!/usr/bin/env python
import rospy
from cv.detectors import dice_detector
from controls.maintainers import yaw_maintainer, depth_maintainer
from controls.controllers import visual_servo

class DiceServo(object):

    def __init__(self, data):

        self.preempted = False

        #TODO: activate
        # self.yaw_maintainer = yaw_maintainer.YawMaintainer()
        # self.depth_maintainer = depth_maintainer.DepthMaintainer()
        self.foundCounts = rospy.get_param("/taskr/dice/foundCounts", 100)
        self.centerStableCounts = rospy.get_param("/taskr/dice/centerStableCounts", 300)
        self.centerMaxError = rospy.get_param("/taskr/dice/centerMaxError", 0.4)
        self.diceName = data["diceName"]


    def start(self, server, feedback_msg):

        # TODO: leave?
        """
        if not self.yaw_maintainer.is_active():
            self.yaw_maintainer.start()

        if not self.depth_maintainer.is_active():
            self.depth_maintainer.start()
        """

        self.diceDetector = dice_detector.DiceDetector(self.diceName)
        self.findDie()
        if self.preempted:
            return
        self.servoDie()


    def stop(self):
        self.preempted = True

        """
        if self.depth_maintainer.is_active():
            self.depth_maintainer.stop()
        if self.yaw_maintainer.is_active():
            self.yaw_maintainer.stop()
        """
        self.diceDetector.stop()

    def findDie(self):
        diceFoundCounts = 0
        while diceFoundCounts < self.foundCounts:

            if self.preempted:
                return

            found = self.diceDetector.foundCorrectDie
            if (found):
                diceFoundCounts += 1
            else:
                diceFoundCounts = 0

            rospy.loginfo("{} / {} frames with {} found".format(
                diceFoundCounts, self.foundCounts,self.diceName))
            rospy.sleep(0.1)

        rospy.loginfo("{} correctly found.".format(self.diceName))


    def servoDie(self):
        #TODO: make sure the low refreshrate is taken into account
        #TODO: change thrust_decay or something
        self.serv = visual_servo.FrontVisualServoController()
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

        rospy.loginfo("Done centering action on {}.".format(self.diceName))
        self.serv.stop()


