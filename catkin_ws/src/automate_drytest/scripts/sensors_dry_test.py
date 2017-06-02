#!/usr/bin/env python
import time
from wait_for_message import *
from std_msgs.msg import Float64

# Eventually we will import more msg type

def check_depth_sensor():
    print("We will begin with the Depth Sensor dry test.")
    check_depth_raw()
    check_depth_pressure()
    print("Depth sensor dry test completed")

def check_depth_raw():
    try:
        wait_for_message("/raw_depth", Float64, 3)

    except Exception:
        print ("> /raw_depth is not being published.")
        pass

def check_depth_pressure():
    try:
        wait_for_message("/depth/pressure", Float64, 3)

    except Exception:
        print ("> /depth/pressure is not being published.")
        pass

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

    elif Agreement == 'yes' or Agreement == 'Yes' or Agreement == 'Y' \
         or Agreement == 'y':
        print("Excellent choice!")
        time.sleep(1)
        check_depth_sensor()

        time.sleep(1)

        """
        print("Moving on to Ximu:")

        try:
            wait_for_message("", Float64, 3)

        except Exception:
            print ("> The Ximu is not connected.")
            pass

        print("Testing the Sonar:")

        try:
            wait_for_message("", Float64, 3)

        except Exception:
            print ("> The Sonar is not connected.")
            pass

        print("Testing the front camera:")

        try:
            wait_for_message("", Float64, 3)

        except Exception:
            print ("> The front camera is not connected.")
            pass
        """

        print('Sensors connection have been tested.')

    else:
        print("I don't know what that means!")
        time.sleep(1)
        print("Try again.")
