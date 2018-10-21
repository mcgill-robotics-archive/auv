#!/usr/bin/env python
import rospy
from cv.detectors import lane_detector
from controls.maintainers import yaw_maintainer, depth_maintainer
from controls.controllers import visual_servo
from controls.utils.utils import normalize_angle

class FollowLane(object):

    def __init__(self, data):

        self.preempted = False

        self.yaw_maintainer = yaw_maintainer.YawMaintainer()
        self.depth_maintainer = depth_maintainer.DepthMaintainer()
        self.foundCounts = rospy.get_param("/taskr/lanes/foundCounts",15)
        self.centerStableCounts = rospy.get_param("/taskr/lanes/centerStableCounts", 50)
        self.centerMaxError = rospy.get_param("/taskr/lanes/centerMaxError", 0.4)
        self.turnStableCounts = rospy.get_param("/taskr/lanes/turnStableCount",50)
        self.turnMaxError = rospy.get_param("/taskr/lanes/turnMaxError",0.1)


    def start(self, server, feedback_msg):
        self.lane_detector = lane_detector.LaneDetector()
        rospy.loginfo("Looking for lane action")


        if not self.yaw_maintainer.is_active():
            self.yaw_maintainer.start()

        if not self.depth_maintainer.is_active():
            self.depth_maintainer.start()

        self.findLane()
        if self.preempted:
            return
        self.servoToCenter()
        if self.preempted:
            return
        self.turn()


    def stop(self):
        self.preempted = True

        if self.depth_maintainer.is_active():
            self.depth_maintainer.stop()
        if self.yaw_maintainer.is_active():
            self.yaw_maintainer.stop()
        #TODO: stop the lane_detector?
        self.lane_detector.stop()

    def findLane(self):

        laneFoundCounts = 0
        while laneFoundCounts < self.foundCounts:

            if self.preempted:
                return

            found = self.lane_detector.getLaneFound()
            if (found):
                laneFoundCounts += 1
            else:
                laneFoundCounts = 0

            rospy.loginfo("{} / {} lane found.".format(
                laneFoundCounts, self.foundCounts))
            rospy.sleep(0.1)

        rospy.loginfo("Lane found.")


    def servoToCenter(self):

        self.serv = visual_servo.DownVisualServoController()
        self.serv.start()

        stable_counts = 0
        while stable_counts < self.centerStableCounts:
            rospy.loginfo("Centering: {} / {} stable periods achieved".format(
                stable_counts,self.centerStableCounts))

            if self.preempted:
                return

            err = self.serv.get_error()
            #debug:
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
        self.serv.stop()

    def turn(self):

        angle = lane_detector.getAngle()

        currentYaw = self.yaw_maintainer.get_current_yaw()
        sp = normalize_angle(currentYaw + angle)

        yaw_maintainer.set_setpoint(sp)

        stable_counts = 0
        while stable_counts < self.turnStableCounts:
            rospy.loginfo("{} / {} stable periods achieved".format(
                stable_counts,self.turnStableCounts))

            if self.preempted:
                return

            err = self.yaw_maintainer.get_error()
            if abs(err) < self.turnMaxError:
                stable_counts += 1
            else:
                stable_counts = 0
            rospy.sleep(0.1)

        rospy.loginfo("Done turn action.")

