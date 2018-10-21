#!/usr/bin/env python
from std_msgs.msg import Int16
import rospy

class Vacuum(object):
    '''Suck or Blow - your choice!'''
    def __init__(self, data):
        self.preempted = False
        self.suckDuration = rospy.get_param("/taskr/vacuum/suckDuration",100)
        self.suckingPower = rospy.get_param("/taskr/vacuum/suckingPower",400)
        self.sucking = data["sucking"]
        if (self.sucking):
            rospy.loginfo("Starting vacuum in sucking mode now!")
        else:
            rospy.loginfo("Starting vacuum in blowing mode now!")

        self.pub = rospy.Publisher('/vacuum',Int16, queue_size=1)

    def start(self, server, feedback_msg):
        self.preempted = False
        feedback_msg.is_done = False
        server.publish_feedback(feedback_msg)

        if (self.sucking) :
            #TODO: Check these values
            msg = self.suckingPower
        else:
            msg = -1 * self.suckingPower

        suckCounts = 0
        while suckCounts < self.suckDuration:

            if (self.preempted):
                return
            self.pub.publish(msg)

            rospy.loginfo("{} / {} publishing suckCommand.".format(
                suckCounts, self.suckDuration))
            suckCounts += 1
            rospy.sleep(0.1)

        self.pub.publish(0)     #stop the vacuum
        rospy.loginfo("Suck completed")

        feedback_msg.is_done = True
        server.publish_feedback(feedback_msg)

    def stop(self):
        """Nothing to do for stop."""
        self.preemted = True
        self.pub.publish(0)     #stop the vacuum
        pass
