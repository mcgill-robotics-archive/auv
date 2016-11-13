#!/usr/bin/env python
import rospy
from rospy import Publisher
from auv_msgs.msg import MotorCommands


def drytest_thrusters():
    for i in range(0, len(thrusters)):
        answer = 'n'
        while(answer.lower() == 'n'):
            print '\nTesting:', thrusters[i]
            raw_input('Press any key to continue')

            for j in range(0, freq):
                pub.publish(create_command(i))
                rate.sleep()
            answer = raw_input('Is it running? y/n: ')
            if answer.lower() == 'y':
                functional[i] = 'Y'
            if answer.lower() == 'n':
                answer = raw_input('Press n to test again and y to move on to the next thruster ')


def create_command(thruster):
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
    freq = 10
    rate = rospy.Rate(freq)

    pub = Publisher("/electrical_interface/motor", MotorCommands, queue_size=5)

    functional = ['N', 'N', 'N', 'N', 'N', 'N', 'N', 'N']
    thrusters = ['Port Bow Heave', 'Bow Sway', 'Starboard Bow Heave',
                 'Starboard Surge', 'Starboard Stern Heave', 'Stern Sway',
                 'Port Stern Heave', 'Port Surge']
    drytest_thrusters()
    print 'Functional Thrusters are: '
    for i in range(0, len(thrusters)):
        if functional[i] == 'Y':
            print "\t", thrusters[i]
    print 'Non-Functional Thrusters are: '
    for i in range(0, len(thrusters)):
        if functional[i] == 'N':
            print "\t", thrusters[i]

    rospy.spin()
