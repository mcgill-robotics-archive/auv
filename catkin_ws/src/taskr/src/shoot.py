#!/usr/bin/env python
import rospy
#import actionlib
from auv_msgs.msg import SolenoidCommands


class Shoot(object):
    """ Shoot a Torpedo from the port (left side) """
    def __init__(self, torpedo):
       rospy.loginfo(torpedo)
       self.torpedoName = torpedo
       

    def start(self, server, feedback_msg):
        pub = rospy.Publisher('/electrical_interface/solenoids', SolenoidCommands, queue_size=10)
        port_torpedo_command = SolenoidCommands()
        port_torpedo_command.port_torpedo = True
        rospy.loginfo(port_torpedo_command)
        pub.publish(port_torpedo_command)

        
