#!/usr/bin/env python
import rospy
from controls.controllers import visual_servo



if __name__ == '__main__':
    rospy.init_node('visual_servo')
    serv = visual_servo.FrontVisualServoController()
    serv.start()
    rospy.spin()

