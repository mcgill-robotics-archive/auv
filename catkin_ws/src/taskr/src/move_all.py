#!/usr/bin/env python
import rospy
from math import fabs
from std_msgs.msg import Float64
from controls.maintainers import YawMaintainer, DepthMaintainer


class MoveAll(object):
    """Move action."""

    VELOCITY = rospy.get_param("taskr/velocity", default=1)
    RATE = rospy.get_param("taskr/vel_cmd_rate", default=10)
    VEL_COEFFICIENT = rospy.get_param("taskr/vel_coefficient", default=1)
    SWAY_VEL_COEFFICIENT = rospy.get_param("taskr/sway_vel_coefficient", default=2)
    MAX_STABLE_COUNTS = rospy.get_param("taskr/max_stable_counts", default=15)
    YAW_THRESH = rospy.get_param("taskr/yaw_threshold", default=0.15)
    DEPTH_THRESH = rospy.get_param("taskr/depth_threshold", default=0.3)

    def __init__(self, point):
        """Constructor for the Move object."""
        self.surge_pub = rospy.Publisher(
            'controls/superimposer/surge', Float64, queue_size=1)
        self.sway_pub = rospy.Publisher(
            'controls/superimposer/sway', Float64, queue_size=1)
        self.preempted = False
        self.distance = point["distance"] if "distance" in point else 0
        self.sway = point["sway"] if "sway" in point else False

        self.yaw = point["yaw"] if "yaw" in point else None
        self.depth = point["depth"] if "depth" in point else None

        if self.yaw is None:
            self.yaw_maintainer = YawMaintainer()
        else:
            self.yaw_maintainer = YawMaintainer(self.yaw)

        if self.depth is None:
            self.depth_maintainer = DepthMaintainer()
        else:
            self.depth_maintainer = DepthMaintainer(self.depth)

    def start(self, server, feedback_msg):
        """Do the move action."""
        rate = rospy.Rate(self.RATE)
        time = self.get_time(fabs(self.distance))
        start = rospy.Time.now()

        surge = self.VELOCITY * self.VEL_COEFFICIENT
        sway = self.VELOCITY * self.SWAY_VEL_COEFFICIENT

        # If the distance is negative, we want to go backwards.
        if self.distance < 0:
            surge *= -0.3  # surge seems to go faster backwards so we only use half the coefficient
            sway *= -1

        # We want to maintain both yaw and depth as we move.
        if not self.yaw_maintainer.is_active():
            self.yaw_maintainer.start()
        if not self.depth_maintainer.is_active():
            self.depth_maintainer.start()

        # First go to depth.
        rospy.loginfo("Trying to go to depth")
        self.wait_for_depth()
        rospy.loginfo("Depth reached.")

        if self.depth is not None:
            if self.depth <= 0.0:
                rospy.loginfo("We're trying to submerge!")
                self.depth_maintainer.stop()
                rospy.sleep(5)
                rospy.loginfo("Hopefully that worked...")
                return

        # Next turn.
        rospy.loginfo("Trying to go to yaw")
        self.wait_for_yaw()
        rospy.loginfo("Yaw reached.")

        # Send surge commands.
        # Should run RATE * TIME times. For example, if we send cmds at
        # 10 cmd/s (Hz), for 5 seconds, we need to loop 50 times.
        for i in range(0, int(self.RATE * time)):
            rospy.loginfo(
                "Sending cmd {}s / {}s".format(float(i) / self.RATE, time))

            if self.preempted:
                return

            if not self.sway:
                self.surge_pub.publish(surge)
            else:
                self.sway_pub.publish(sway)
            rate.sleep()

        # Sleep is needed to allow robot to stop before other actions are done.
        rospy.sleep(2)

        rospy.loginfo(
            "Done move in time {}".format((rospy.Time.now() - start).to_sec()))

    def get_time(self, distance):
        """Get the time for which to travel at the given velocity to achieve
        desired distance."""
        return distance / self.VELOCITY * 2

    def stop(self):
        self.preempted = True
        self.surge_pub.publish(0)
        self.sway_pub.publish(0)

        if self.depth_maintainer.is_active():
            self.depth_maintainer.stop()
        if self.yaw_maintainer.is_active():
            self.yaw_maintainer.stop()

    def wait_for_depth(self):
        stable_counts = 0
        while stable_counts < self.MAX_STABLE_COUNTS:
            if self.preempted:
                return

            err = self.depth_maintainer.error
            if err is None:
                pass
            elif abs(err) < self.DEPTH_THRESH:
                rospy.loginfo("{} / {} stable depth periods".format(stable_counts, self.MAX_STABLE_COUNTS))
                stable_counts += 1
            else:
                stable_counts = 0
            rospy.sleep(0.1)

    def wait_for_yaw(self):
        stable_counts = 0
        while stable_counts < self.MAX_STABLE_COUNTS:
            if self.preempted:
                return

            err = self.yaw_maintainer.get_error()
            if abs(err) < self.YAW_THRESH:
                stable_counts += 1
                rospy.loginfo("{} / {} stable yaw periods".format(stable_counts, self.MAX_STABLE_COUNTS))
            else:
                stable_counts = 0
            rospy.sleep(0.1)
