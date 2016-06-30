#!/usr/bin/env python
import rospy
from actionlib import SimpleActionServer
from auv_msgs.msg import SetVelocityAction, SetVelocityFeedback, SetVelocityResult


class FakeControls(object):

    """FakeControls object.

    Do not run this at the same time as controls. Only for testing.
    """

    def __init__(self):
        """Constructor for FakeControls."""
        # Make a server named "controls_velocity".
        self.server = SimpleActionServer(
            "controls_velocity", SetVelocityAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()

    def execute_cb(self, goal):
        """Callback for goal."""
        print "Controls goal!", goal

        rate = rospy.Rate(1)

        # Garbage feedback.
        feedback = SetVelocityFeedback(yaw_error=1, depth_error=1)
        self.server.publish_feedback(feedback)
        print "Feedback published"

        rate.sleep()

        # Positive result.
        result = SetVelocityResult(success=True)
        self.server.set_succeeded(result)
        print "Done"


if __name__ == '__main__':
    rospy.init_node("fake_controls_server")
    FakeControls()
    rospy.spin()
