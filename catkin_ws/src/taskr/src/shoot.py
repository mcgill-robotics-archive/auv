#!/usr/bin/env python
import rospy
from auv_msgs.msg import SolenoidCommands

#Shoot a torpedo from either port or starboard side
class Shoot(object):
    def __init__(self, torpedo):
       rospy.loginfo("Shoot {0}!" .format(torpedo))
       self.torpedoName = torpedo
       

    def start(self, server, feedback_msg):
        feedback_msg.is_done = False
        server.publish_feedback(feedback_msg)

        pub = rospy.Publisher('/electrical_interface/solenoids', SolenoidCommands, queue_size=1)
        torpedo_command = SolenoidCommands()

        # Set the attribute of torpedo_command specified in the yaml
        # file (port_torpedo or starboard_torpedo) to True
        # Use rostopic echo /electrical_interface/solenoids to debug the pub
        setattr(torpedo_command, self.torpedoName, True)

        # Put preempted for good measure.
        if server.is_preempt_requested():
            rospy.loginfo("Shoot preempted")
            server.set_preempted()
            return
        pub.publish(torpedo_command)

        feedback_msg.is_done = True
        server.publish_feedback(feedback_msg)

