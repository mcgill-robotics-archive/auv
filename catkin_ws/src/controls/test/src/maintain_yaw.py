#! /usr/bin/env python

import rospy
import time

from std_msgs.msg import Float64

from controls.servo_controller import YawMaintainer

if __name__ == '__main__':
    rospy.init_node('maintain_yaw') 
    ym = YawMaintainer(1.0)

    ym.start()
    '''
    stable_counts = 0
    while stable_counts < 20:
        err = ym.get_error()
        print("error", err)
        if abs(err) < 0.2:
            stable_counts += 1
        else:
            stable_counts = 0
        print("stable_counts", stable_counts)
        rospy.sleep(0.1)
    ym.stop()
    '''

    rospy.spin()
