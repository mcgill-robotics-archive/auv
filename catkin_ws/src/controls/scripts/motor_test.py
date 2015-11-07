#!/usr/bin/env python

import math
import rospy
from auv_msgs.msg import MotorCommands


def iter_cmd():
    t = 0

    cmd = MotorCommands()
    while not rospy.is_shutdown():
        next_surge = 500 * math.sin(t * 2 * math.pi / 360)
        t += 1
        cmd.port_surge = next_surge
        cmd.starboard_surge = next_surge
        yield cmd


if __name__ == '__main__':
    rospy.init_node("test_motors")
    pub = rospy.Publisher("/electrical_interface/motor", MotorCommands, queue_size=3)
    
    rate = rospy.Rate(20)
    for cmd in iter_cmd():
        pub.publish(cmd)
        rate.sleep()
