#!/usr/bin/env python
from auv_msgs.msg import SolenoidCommands

import rospy


class Shoot(object):
    '''Shoot a projectile'''
    def __init__(self, projectile):
        rospy.loginfo("Shoot {} now!".format(projectile))
        self.projectile_name = projectile
        self.pub = rospy.Publisher('/electrical_interface/solenoid',
                                   SolenoidCommands, queue_size=1)

    def start(self, server, feedback_msg):
        feedback_msg.is_done = False
        server.publish_feedback(feedback_msg)

        solenoid_command = SolenoidCommands()

        # Set the attribute of solenoid_command specified in the yaml
        # file to True
        setattr(solenoid_command, self.projectile_name, True)

        # Handle actionlib server preemption
        if server.is_preempt_requested():
            rospy.loginfo("Shoot preempted")
            server.set_preempted()
            return

        while self.pub.get_num_connections() == 0:
            rospy.logdebug("Waiting for subscribers in Shoot")
            continue

        self.pub.publish(solenoid_command)

        # Sleep for allowing teensy and pneumatics to react
        rospy.sleep(0.2)

        feedback_msg.is_done = True
        server.publish_feedback(feedback_msg)
