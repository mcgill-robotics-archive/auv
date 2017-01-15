#!/usr/bin/env python
import rospy
import numpy
from math import fabs
from actionlib import SimpleActionClient
from geometry_msgs.msg import Point, Vector3Stamped
from auv_msgs.msg import SetVelocityAction, SetVelocityGoal
from std_msgs.msg import Float64


class Move(object):
    """Move action."""

    VELOCITY = rospy.get_param("taskr/velocity", default=1)
    RATE = rospy.get_param("taskr/vel_cmd_rate", default=10)
    VEL_COEFFICIENT = rospy.get_param("taskr/vel_coefficient", default=1)
    USE_FEEDBACK = rospy.get_param("taskr/use_feedback")
    ERROR_THRESHOLD = rospy.get_param("taskr/yaw_error_threshold", default=numpy.pi / 8)

    def __init__(self, point):
        """Constructor for the Move object."""
        self.curr_yaw = None
        self.curr_depth = None

        self.depth_sub = rospy.Subscriber('state_estimation/depth', Float64, self.depth_callback)
        self.pose_sub = rospy.Subscriber('robot_state', Vector3Stamped, self.pose_callback)

        self.distance = point["distance"]

        self.sway = point["sway"] if "sway" in point else False

        self.depth = point["depth"] if "depth" in point else None
        self.yaw = point["yaw"] if "yaw" in point else None

        # Whether to move forward or backwards
        self.forward = (self.distance >= 0)

        # Whether to get yaw feedback from sensors.
        self.feedback = (point["feedback"] if "feedback" in point else False) and self.USE_FEEDBACK

        # If feedback is to be used, subscribe to sonar.
        if self.feedback:
            rospy.loginfo("Move with feedback requested")
            self.sonar_sub = rospy.Subscriber("/sonar_proc/task_point", Point, self.sonar_callback)
            self.sonar_correction = 0

        # Create velocity action client for controls server.
        self.vel_client = SimpleActionClient("controls_velocity", SetVelocityAction)
        self.vel_client.wait_for_server()

    def start(self, server, feedback_msg):
        """Do the move action.

        Args:
            server: Action server for publishing feedback.
            feedback_msg: Feedback message to populate.
        """
        rate = rospy.Rate(self.RATE)

        ctrl_goal = SetVelocityGoal()

        # If depth was not set, take the current depth.
        if self.depth is None:
            while self.curr_depth is None and not rospy.is_shutdown():
                rate.sleep()

            ctrl_goal.cmd.depth = self.curr_depth
        else:
            ctrl_goal.cmd.depth = self.depth

        # If yaw was not set, take the current yaw.
        if self.yaw is None:
            while self.curr_yaw is None and not rospy.is_shutdown():
                rate.sleep()

            ctrl_goal.cmd.yaw = self.curr_yaw
        else:
            ctrl_goal.cmd.yaw = self.yaw

        time = self.get_time(fabs(self.distance))

        # Send yaw and depth goal without velocity first.
        rospy.loginfo("Sending initial yaw and depth only")
        # Send to velocity server
        self.vel_client.send_goal(ctrl_goal)
        # Check if we received preempt request from Taskr.
        if server.is_preempt_requested():
            rospy.logerr("Move preempted")
            # Send preempt request to Controls
            self.vel_client.cancel_goal()
            server.set_preempted()
            return

        self.vel_client.wait_for_result()

        if not self.sway:
            ctrl_goal.cmd.surgeSpeed = self.VELOCITY * self.VEL_COEFFICIENT
            rospy.loginfo("Surge command received!")
        else:
            ctrl_goal.cmd.swaySpeed = self.VELOCITY * self.VEL_COEFFICIENT
            rospy.loginfo("Sway command received!")

        if not self.forward:
            ctrl_goal.cmd.surgeSpeed *= -1
            ctrl_goal.cmd.swaySpeed *= -1

        start = rospy.Time.now()

        # Send surge commands.
        # Should run RATE * TIME times. For example, if we send cmds at
        # 10 cmd/s (Hz), for 5 seconds, we need to loop 50 times.
        for i in range(0, int(self.RATE * time)):
            print "Sending cmd", float(i) / self.RATE, "s /", time, "s"

            # Only if feedback is being used, correct yaw.
            if self.feedback and self.sonar_correction != 0:
                rospy.loginfo("Correcting sonar by {}".format(self.sonar_correction))
                # Correct the yaw and then reset.
                new_yaw = ctrl_goal.cmd.yaw - self.sonar_correction
                # Wrap between -pi and pi
                ctrl_goal.cmd.yaw = (new_yaw + numpy.pi) % (2 * numpy.pi) - numpy.pi
                self.sonar_correction = 0

            self.vel_client.send_goal(ctrl_goal)

            # Check if we received preempt request from Planner
            if server.is_preempt_requested() or rospy.is_shutdown():
                rospy.logerr("Taskr preempted")
                # Send preempt request to Controls
                self.vel_client.cancel_goal()
                server.set_preempted()
                return

            feedback_msg.is_done = False  # Not super useful feedback.
            server.publish_feedback(feedback_msg)

            rate.sleep()

        rospy.loginfo("Done move in time {}".format((rospy.Time.now() - start).to_sec()))

    def get_time(self, distance):
        """Get the time for which to travel at the given velocity to achieve
        desired distance."""
        return distance / self.VELOCITY

    def sonar_callback(self, goal):
        """Callback for the sonar topic. Calculates the difference between
        sonar's perceived goal and the current yaw and populates the associated
        correction value accordingly."""
        error = numpy.arctan2(goal.y, goal.x)

        # Only account for this error if it seems valid, ie is under a given
        # threshold.
        if error < self.ERROR_THRESHOLD:
            rospy.loginfo("Correction seems valid, shifting by {}".format(error))
            self.sonar_correction = error

    def depth_callback(self, msg):
        self.curr_depth = msg.data

    def pose_callback(self, msg):
        self.curr_yaw = msg.vector.z
