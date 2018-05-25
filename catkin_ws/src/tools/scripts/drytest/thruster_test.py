#!/usr/bin/env python

"""
Thruster Dry Test

This cycles through each thruster turning it on and asking the user to
confirm functionality.
"""

import rospy
from rospy import Publisher

from auv_msgs.msg import ThrusterCommands
from console_format import format


class ThrusterTest:

    def __init__(self):
        self.cmd = ThrusterCommands()

        # Magnitude of the pulse sent to the thrusters
        self.pulse = 100

        # Message Frequency
        self.freq = 10

        # Dictionary of thrusters indexed by the message definitions
        self.thrusters = {
            self.cmd.SURGE_PORT: 'Port Surge',
            self.cmd.SURGE_STARBOARD: 'Surge Starboard',
            self.cmd.SWAY_BOW: 'Bow Sway',
            self.cmd.SWAY_STERN: 'Stern Sway',
            self.cmd.HEAVE_BOW_PORT: 'Port Bow Heave',
            self.cmd.HEAVE_BOW_STARBOARD: 'Starboard Bow Heave',
            self.cmd.HEAVE_STERN_PORT: 'Port Stern Heave',
            self.cmd.HEAVE_STERN_STARBOARD: 'Starboard Stern Heave'
        }

        # Stores the results of the test
        self.results = {
            'func', [],
            'nfunc', []
        }

        self.pub = Publisher('electrical_interface/thrusters',
                             ThrusterCommands,
                             queue_size=5)

    def drytest_thrusters(self):

        rate = rospy.Rate(self.freq)
        is_working = False

        for i in range(0, len(self.thrusters)):
            answer = 'n'

            print('\n' + format.UNDERLINE + format.OKBLUE + self.thrusters[i] +
                  format.ENDC)

            # Tests the same thruster until answer is changed to y
            while(answer.lower() == 'n'):
                raw_input('Press enter to send a pulse ')

                # Publish a command at the specified frequency
                self.set_command(i)
                for j in range(0, self.freq):
                    self.pub.publish(self.cmd)
                    rate.sleep()

                answer = raw_input('Is it running? [y/N]: ')

                # Adds thruster to func results if yes
                if answer.lower() == 'y':
                    is_working = True

                # Ask to try again if anything else
                else:
                    answer = raw_input('Enter n to try again and any other '
                                       'key to move to the next thruster ')

            if (is_working):
                self.results['func'].append(self.thrusters[i])
                print (format.OKGREEN + format.BOLD +
                       'The thruster is working!' + format.ENDC)
            else:
                self.results['nfunc'].append(self.thrusters[i])
                print (format.FAIL + format.BOLD + 'The thruster is not '
                       'responsive' + format.ENDC)

    def set_command(self, thruster):
        self.cmd.thruster_commands = [0, 0, 0, 0, 0, 0, 0, 0]
        self.cmd.thruster_commands[thruster] = self.pulse

    def run_test(self):
        isError = False

        print (format.OKBLUE + format.BOLD + '\n\n'
               ' #######################\n'
               ' ## TESTING THRUSTERS ##\n'
               ' #######################\n' + format.ENDC)

        self.drytest_thrusters()

        # SUMMARY -------------------------------------------------------------
        print('\n' + format.UNDERLINE + format.OKBLUE + 'Summary' +
              format.ENDC)

        # Prints All func Thrusters
        print ('\n' + format.OKGREEN + format.BOLD +
               'Functional Thrusters are: ')
        for i in range(0, len(self.results['func'])):
                print (" - " + self.results['func'][i])
        print (format.ENDC)

        # Prints All Non-func Thrusters
        print ('\n' + format.FAIL + format.BOLD +
               'Non-functional Thrusters are: ')
        for i in range(0, len(self.results['nfunc'])):
                print (' - ' + self.results['nfunc'][i])
                isError = True
        print (format.ENDC)

        print (format.OKBLUE + '\nFinished testing thrusters\n\n' +
               format.ENDC)

        return isError
