#!/usr/bin/env python
import time
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
    print("Are you ready to begin the dry test? Yes? No?")

    Agreement = raw_input()

    if Agreement == 'no' or Agreement == 'No':
        print ("Maybe later. Goodbye!")

    elif Agreement == 'yes' or Agreement == 'Yes':
        print("Excellent choice!")
        time.sleep(1)
        print("We will begin with the Depth Sensor dry test.")

        try:
            wait_for_message("/raw_depth", Float64, 3)

        except Exception:
            print ("> The Depth Sensor is not connected.")
            pass

        # time.sleep(1)

        # print("Moving on to Sonar:")

        # try:
            # wait_for_message("/raw_depth", Float64, 3)

        # except Exception:
            # print ("> The Sonar is not connected.")
            # pass

    else:
        print("I don't know what that means!")
        time.sleep(1)
        print("Try again.")
