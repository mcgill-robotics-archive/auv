#!/usr/bin/env python
import rospy
from math import fabs
from std_msgs.msg import Float64
from controls.servo_controller import YawMaintainer, DepthMaintainer

"""
TODO: Jana
- Make this work for move backwards and in sway.
- Publish feedback
"""


class Move(object):
    """Move action."""

    VELOCITY = rospy.get_param("taskr/velocity", default=1)
    RATE = rospy.get_param("taskr/vel_cmd_rate", default=10)
    VEL_COEFFICIENT = rospy.get_param("taskr/vel_coefficient", default=1)

    def __init__(self, point):
        """Constructor for the Move object."""
        self.surge_pub = rospy.Publisher(
            'controls/superimposer/surge', Float64, queue_size=1)
        self.preempted = False
        self.distance = point["distance"]

    def start(self, server, feedback_msg):
        """Do the move action."""
        rate = rospy.Rate(self.RATE)
        time = self.get_time(fabs(self.distance))
        start = rospy.Time.now()

        surge = self.VELOCITY * self.VEL_COEFFICIENT

        # If the distance is negative, we want to go backwards.
        if self.distance < 0:
            surge *= -1

        self.surge_pub.publish(surge)
        yaw_maintainer = YawMaintainer()
        yaw_maintainer.start()
        depth_maintainer = DepthMaintainer()
        depth_maintainer.start()

        # Send surge commands.
        # Should run RATE * TIME times. For example, if we send cmds at
        # 10 cmd/s (Hz), for 5 seconds, we need to loop 50 times.
        for i in range(0, int(self.RATE * time)):
            rospy.loginfo(
                "Sending cmd {}s / {}s".format(float(i) / self.RATE, time))

            if self.preempted:
                yaw_maintainer.stop()
                depth_maintainer.stop()
                return

            self.surge_pub.publish(surge)
            rate.sleep()

        # Sleep is needed to allow robot to stop before other actions are done.
        rospy.sleep(2)

        yaw_maintainer.stop()
        depth_maintainer.stop()
        rospy.loginfo(
            "Done move in time {}".format((rospy.Time.now() - start).to_sec()))

    def get_time(self, distance):
        """Get the time for which to travel at the given velocity to achieve
        desired distance."""
        return distance / self.VELOCITY

    def stop(self):
        self.preempted = True
