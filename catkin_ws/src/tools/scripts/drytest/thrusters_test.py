#!/usr/bin/env python

"""
Thruster Dry Test.

This cycles through each thruster turning it on and asking the user to
confirm functionality.
"""

import rospy
from rospy import Publisher

from auv_msgs.msg import ThrusterCommands
from console_format import format


def drytest_thrusters():
    pub = Publisher("electrical_interface/motor",
                    ThrusterCommands,
                    queue_size=5)

    """Cycles through each thruster and asks for user confirmation"""

    # Keeps track of which thrusters are functional
    functional = ['N', 'N', 'N', 'N', 'N', 'N', 'N', 'N']

    # Thruster names
    thrusters = ['Port Surge', 'Starboard Surge', 'Bow Sway',
                 'Stern Sway', 'Port Bow Heave', 'Starboard Bow Heave',
                 'Port Stern Heave', 'Starboard Stern Heave']

    freq = 10  # set the frequency
    rate = rospy.Rate(freq)  # set the rate

    for i in range(0, len(thrusters)):
        answer = 'n'

        print('\n' + format.UNDERLINE + format.OKBLUE + thrusters[i] +
              format.ENDC)

        # Tests the same thruster until answer is changed to y
        while(answer.lower() == 'n'):
            raw_input('Press any key to send pulse ')

            # publish this command at the specified frequency
            for j in range(0, freq):
                pub.publish(create_command(i))
                rate.sleep()
            answer = raw_input('Is it running? y/n: ')

            # moves on to the next thruster if the answer is yes
            if answer.lower() == 'y':
                functional[i] = 'Y'

            # try again if the answer is no
            else:
                answer = raw_input('Press n to test again and any other '
                                   'key to move to the next thruster ')

        if (functional[i] == 'Y'):
            print (format.OKGREEN + format.BOLD + 'The thruster is working!' +
                   format.ENDC)
        else:
            print (format.FAIL + format.BOLD + 'The thruster is not '
                   'responsive' + format.ENDC)

    return functional, thrusters


def create_command(t):
    """This function turns on one thruster and sets the rest to 0"""
    cmd = ThrusterCommands()
    cmd.thruster_commands[cmd.SURGE_PORT] = 300 if t == 0 else 0
    cmd.thruster_commands[cmd.SURGE_STARBOARD] = 300 if t == 1 else 0
    cmd.thruster_commands[cmd.SWAY_BOW] = 300 if t == 2 else 0
    cmd.thruster_commands[cmd.SWAY_STERN] = 300 if t == 3 else 0
    cmd.thruster_commands[cmd.HEAVE_BOW_PORT] = 300 if t == 4 else 0
    cmd.thruster_commands[cmd.HEAVE_BOW_STARBOARD] = 300 if t == 5 else 0
    cmd.thruster_commands[cmd.HEAVE_STERN_PORT] = 300 if t == 6 else 0
    cmd.thruster_commands[cmd.HEAVE_STERN_STARBOARD] = 300 if t == 7 else 0
    return cmd


def run_test():
    isAllGood = True

    print (format.OKBLUE + format.BOLD + '\n\n'
           ' #######################\n'
           ' ## TESTING THRUSTERS ##\n'
           ' #######################\n' + format.ENDC)

    functional, thrusters = drytest_thrusters()

    # SUMMARY -----------------------------------------------------------------
    print('\n' + format.UNDERLINE + format.OKBLUE + 'Summary' +
          format.ENDC)

    # Prints All Functional Thrusters
    print ('\n' + format.OKGREEN + format.BOLD + 'Functional Thrusters are: ')
    for i in range(0, len(thrusters)):
        if functional[i] == 'Y':
            print (" - " + thrusters[i])
    print (format.ENDC)

    # Prints All Non-Functional Thrusters
    print ('\n' + format.FAIL + format.BOLD + 'Non-functional Thrusters are: ')
    for i in range(0, len(thrusters)):
        if functional[i] == 'N':
            print (' - ' + thrusters[i])
            isAllGood = False
    print (format.ENDC)

    print (format.OKBLUE + '\nFinished testing thrusters\n\n' + format.ENDC)

    if (not isAllGood):
        return True  # >> isError
    else:
        return False  # >> is Error


if __name__ == "__main__":
    rospy.init_node("drytest_thruster")
    run_test()
