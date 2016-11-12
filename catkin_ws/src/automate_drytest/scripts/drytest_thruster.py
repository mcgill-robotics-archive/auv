#!/usr/bin/env python
import rospy
from rospy import Publisher
from auv_msgs.msg import MotorCommands
# from graphics import *


def drytest_thrusters():
    for i in range(0, len(thrusters)):
        print '\nTesting:', thrusters[i]
        # draw_thrusters(i)
        raw_input('Press any key to continue')

        for j in range(0, freq):
            pub.publish(create_command(i))
            rate.sleep()
        answer = raw_input('Is it running? y/n: ')
        if answer.lower() == 'y':
            functional[i] = 'Y'


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


def draw_thrusters(thruster_num):
    top = Line(Point(60, 30), Point(120, 30))
    left = Line(Point(60, 30), Point(60, 120))
    bottom = Line(Point(60, 120), Point(120, 120))
    right = Line(Point(120, 120), Point(120, 30))
    top.draw(win)
    left.draw(win)
    bottom.draw(win)
    right.draw(win)
    thr1 = Circle(Point(60, 30), 10)
    thr2 = Cirle(Point(90, 30), 10)
    thr3 = Cirle(Point(120, 30), 10)
    thr4 = Cirle(Point(120, 75), 10)
    thr5 = Cirle(Point(120, 120), 10)
    thr6 = Cirle(Point(90, 120), 10)
    thr7 = Cirle(Point(60, 120), 10)
    thr8 = Cirle(Point(60, 75), 10)
    thr1.draw(win)
    thr2.draw(win)
    thr3.draw(win)
    thr4.draw(win)
    thr5.draw(win)
    thr6.draw(win)
    thr7.draw(win)
    thr8.draw(win)
    if thruster_num == 1:
        thr1.setFill('blue')
    if thruster_num == 2:
        thr2.setFill('blue')
    if thruster_num == 3:
        thr3.setFill('blue')
    if thruster_num == 4:
        thr4.setFill('blue')
    if thruster_num == 5:
        thr5.setFill('blue')
    if thruster_num == 6:
        thr6.setFill('blue')
    if thruster_num == 7:
        thr7.setFill('blue')
    if thruster_num == 8:
        thr8.setFill('blue')


if __name__ == "__main__":
    rospy.init_node("drytest_thruster")
    freq = 10
    rate = rospy.Rate(freq)

    pub = Publisher("/electrical_interface/motor", MotorCommands, queue_size=5)

    # win = GraphWin('Bradbury', 200, 200)

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
