#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String

"""Depth sensor self test

We aquire the data published to /state_estimation/raw_depth topic by raw_pub

We could as well aquire the pressure data from the depth sensor via topic 
/depth_sensor/pressure, but since we are interested in distances I thought it 
would be more representative. 

"""

def callback(msg):
"""Callback: Print the depth of the robot relative to a position at which the 
   pressure is 101.325 kPa. 

Args: 
    msg: Float64. Corresponds to raw_depth value. 

"""
    print("The depth sensor readings are:\n")
    return(msg)

def depth_sensor_self_test():
    rospy.init_node("depth_sensor_self_test", anonymous=True)
    rospy.Subscriber("/state_estimation/raw_depth", Float64, callback)
    rospy.spin()

def start_drytest():
    """Ask for permission to begin the drytest"""

    begin = str(raw_input('Begin Depth sensor drytest? Y/N'))

    if begin != 'Y' and begin != '':
        print('Maybe later.')
        return None

    try:
        depth_sensor_self_test():
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    start_drytest()

