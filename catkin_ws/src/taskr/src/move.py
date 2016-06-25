#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import Point
from auv_msgs.msg import SetVelocityAction, SetVelocityGoal


class Move(object):
    """Move action."""

    VELOCITY = 5  # Apparently "no units"

    def __init__(self, point):
        """Constructor for the Move object."""
        self.goal = Point(x=point["position"]["x"],
                          y=point["position"]["y"],
                          z=point["position"]["z"])
        self.yaw = point["yaw"] if "yaw" in point else None

        # Whether to get yaw feedback from sensors. Not yet implemented.
        self.feedback = point["feedback"] if "feedback" in point else False

        # Create velocity action client for controls server.
        self.vel_client = SimpleActionClient("controls_velocity", SetVelocityAction)
        self.vel_client.wait_for_server()

    def start(self, server, feedback_msg):
        """Do the move action.

        Args:
            server: Action server for publishing feedback.
            feedback_msg: Feedback message to populate.
        """
        rate = rospy.Rate(10)

        ctrl_goal = SetVelocityGoal()
        ctrl_goal.cmd.depth = self.goal.z
        ctrl_goal.cmd.yaw = self.yaw

        time = self.getTime()

        print time

        # Send yaw goal without velocity first.
        if self.yaw != 0:
            print "Sending yaw"
            # Send to velocity server
            self.vel_client.send_goal(ctrl_goal)
            # Check if we received preempt request from Taskr.
            if server.is_preempt_requested():
                print "Move preempted"
                # Send preempt request to Controls
                self.vel_client.cancel_goal()
                self._as.set_preempted()
                return

            self.vel_client.wait_for_result()

        ctrl_goal.cmd.surgeSpeed = self.VELOCITY

        # Send surge commands.
        for s in range(1, time * 10 + 1, 1):  # No idea what this is.
            print "Sending Surge"
            self.vel_client.send_goal(ctrl_goal)
            # Check if we received preempt request from Planner
            if server.is_preempt_requested():
                print "Taskr preempted"
                # Send preempt request to Controls
                self.vel_client.cancel_goal()
                self._as.set_preempted()
                return

            feedback_msg.is_done = False  # Not super useful feedback.
            server.publish_feedback(feedback_msg)

            # Sleep for the amount of time that makes the for-loop run
            # for 1 second.
            rate.sleep()

        print "Done move."

    def get_time(self):
        """Get the time for which to travel at the given velocity to achieve
        desired distance."""
        dist = pow(pow(self.goal.x, 2) + pow(self.goal.y, 2), 0.5)
        return int(dist / self.VELOCITY)
