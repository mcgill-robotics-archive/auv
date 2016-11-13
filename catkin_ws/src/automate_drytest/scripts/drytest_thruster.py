#!/usr/bin/env python

"""Thruster Dry Test.

This cycles through each thruster turning it on and asking the user to
confirm functionality.
"""

import rospy
from rospy import Publisher
from auv_msgs.msg import MotorCommands


def drytest_thrusters():
    """Cycles through each thruster and asks for user confirmation"""
    for i in range(0, len(thrusters)):
        # sets answer back to no for the new thruster
        answer = 'n'
        # Tests the same thruster until answer is changed to y
        while(answer.lower() == 'n'):
            print '\nTesting:', thrusters[i]
            raw_input('Press any key to continue')

            for j in range(0, freq):  # publish this command at the specified frequency
                pub.publish(create_command(i))
                rate.sleep()
            answer = raw_input('Is it running? y/n: ')
            # moves on to the next thruster if the answer is yes
            if answer.lower() == 'y':
                functional[i] = 'Y'
            # try again if the answer is no
            else:
                answer = raw_input('Press n to test again and y to move on '
                                   'to the next thruster ')


def create_command(thruster):
    """This function turns on one thruster and sets the rest to 0"""
    cmd = MotorCommands()
    cmd.port_bow_heave = 300 if thruster == 0 else 0
    cmd.bow_sway = 300 if thruster == 1 else 0
    cmd.starboard_bow_heave = 300 if thruster == 2 else 0
    cmd.starboard_surge = 300 if thruster == 3 else 0
    cmd.starboard_stern_heave = 300 if thruster == 4 else 0
    cmd.stern_sway = 300 if thruster == 5 else 0
    cmd.port_stern_heave = 300 if thruster == 6 else 0
    cmd.port_surge = 300 if thruster == 7 else 0
    return cmd


if __name__ == "__main__":
    rospy.init_node("drytest_thruster")
    freq = 10  # set the frequency
    rate = rospy.Rate(freq)  # set the rate

    pub = Publisher("/electrical_interface/motor", MotorCommands, queue_size=5)

    # Keeps track of which thrusters are functional, index matches that of thrusters
    functional = ['N', 'N', 'N', 'N', 'N', 'N', 'N', 'N']
    # Thruster names
    thrusters = ['Port Bow Heave', 'Bow Sway', 'Starboard Bow Heave',
                 'Starboard Surge', 'Starboard Stern Heave', 'Stern Sway',
                 'Port Stern Heave', 'Port Surge']
    drytest_thrusters()

    # prints a list of functional and non-functional thrusters
    print 'Functional Thrusters are: '
    for i in range(0, len(thrusters)):
        if functional[i] == 'Y':
            print "\t", thrusters[i]
    print 'Non-Functional Thrusters are: '
    for i in range(0, len(thrusters)):
        if functional[i] == 'N':
            print "\t", thrusters[i]

    rospy.spin()
