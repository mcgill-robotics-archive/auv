#!/usr/bin/env python
import rospy
from auv_msgs.msg import SolenoidCommands


class Shoot(object):
    """ Shoot a Torpedo from the port (left side) """
    def __init__(self, torpedo):
       rospy.loginfo(torpedo)
       self.torpedoName = torpedo
       

    def start(self, server, feedback_msg):
        feedback_msg.is_done = False
        server.publish_feedback(feedback_msg)

        pub = rospy.Publisher('/electrical_interface/solenoids', SolenoidCommands, queue_size=10)
        torpedo_command = SolenoidCommands()

        # Set the attribute of torpedo_command specified in the yaml
        # file (port_torpedo or starboard_torpedo) to True
        setattr(torpedo_command, self.torpedoName, True)

        rospy.loginfo(torpedo_command)
        
        if server.is_preempt_requested():
            rospy.loginfo("Shoot preempted")
            server.set_preempted()
            return
        pub.publish(torpedo_command)

        feedback_msg.is_done = True
        server.publish_feedback(feedback_msg)

