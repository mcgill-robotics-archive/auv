#!/usr/bin/env python
import rospy
from auv_msgs.msg import MotorCommands


def callback(data):
    # Sorry
    rospy.loginfo(str(rospy.get_name()+ "port_surge: {} starboard_surge: {} bow_sway: {} stern_sway: {} port_bow_heave: {} starboard_bow_heave: {} port_stern_heave:{} starboard_stern_heave: {}").format(data.port_surge, data.starboard_surge, data.bow_sway, data.stern_sway, data.port_bow_heave, data.starboard_bow_heave, data.port_stern_heave, data.starboard_stern_heave))


def listener():
    rospy.init_node('drytest_listener', anonymous=True)
    rospy.Subscriber('/electrical_interface/motor', MotorCommands, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
