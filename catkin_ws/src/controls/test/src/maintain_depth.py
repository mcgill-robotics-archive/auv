#! /usr/bin/env python

import rospy
import time

from std_msgs.msg import Float64

from controls.servo_controller import DepthMaintainer

if __name__ == '__main__':
    rospy.init_node('maintain_depth') 
    dm = DepthMaintainer(2.0)

    dm.start()
    time.sleep(10)
    dm.stop()

    pub = rospy.Publisher('controls/superimposer/heave', Float64)
    pub.publish(0)
    rospy.spin()
