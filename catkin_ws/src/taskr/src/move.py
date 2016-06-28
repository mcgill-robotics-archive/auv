#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from auv_msgs.msg import SetVelocityAction, SetVelocityGoal


class Move(object):
    """Move action."""

    VELOCITY = rospy.get_param("taskr/velocity", default=50)  # Apparently "no units"

    def __init__(self, point):
        """Constructor for the Move object."""
        self.distance = point["distance"]
        self.depth = point["depth"]
        self.yaw = point["yaw"]
        self.rate = 10

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
        rate = rospy.Rate(self.rate)

        ctrl_goal = SetVelocityGoal()
        ctrl_goal.cmd.depth = self.depth
        ctrl_goal.cmd.yaw = self.yaw

        time = self.get_time(self.distance)

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

        start = rospy.Time.now()

        # Send surge commands.
        # Should run RATE * TIME times. For exmaple, if we send cmds at
        # 10 cmd/s (Hz), for 5 seconds, we need to loop 50 times.
        for i in range(0, int(self.rate * time)):
            print "Sending Surge", float(i) / self.rate, "s /", time, "s"
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

            rate.sleep()

        print "Done move in time", (rospy.Time.now() - start).to_sec()

    def get_time(self, distance):
        """Get the time for which to travel at the given velocity to achieve
        desired distance."""
        print distance, "/", self.VELOCITY
        return distance / self.VELOCITY
