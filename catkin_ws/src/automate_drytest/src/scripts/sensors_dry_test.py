#!/usr/bin/env python

from wait_for_message import *
from std_msgs.msg import Float64

# Eventually we will import more msg type

if __name__ == "__main__":

    """Calls function wait_for_message which creates an object for each sensor
       being tested.

       Waits for message:
       - "None": Sensor is not connected
       -  Data published to the topic : Sensor is connected

    """
    rospy.init_node("Testing")
    print("Beginning of the sensors dry test.")
    print("Depth sensor:")
    try:
        wait_for_message("/raw_depth", Float64, 3)

    except Exception:
        print ("Sensor is not connected.")
        pass

    # print("Moving on to ____ test:")
    # try:
        # wait_for_message("/raw_depth", Float64, 3)

    # except Exception:
        # print ("Sensor not connected.")
        # pass
