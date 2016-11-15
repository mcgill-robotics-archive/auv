#**AUTOMATE DRYTEST**

##**Test**

###*Sensors Dry Test*

  *sensors_dry_test.py* together with *wait_for_message.py* detect
   whether or not given sensors are connected to the robot.


  *sensors_dry_test.py* imports the *wait_for_message.py* classes
   library. It then defines a *Testing* node and call the function
   wait_for_message inside *wait_for_message.py* with the topic that is
   being tested and the msg type.


  *wait_for_for message.py* defines the node as a Subscriber and
  subsribes to the topic. It detect whether or not the selected sensor
  is connected to the robot.

  A message is returned to *sensors_dry_test.py*:

  >If the sensor is not detected: Returns "None"

  >If the sensor is detected: Returns data published by the sensor
    inside the topic.

Every connection is tested within a finite time thanks to a timer. The value of Timout can be modified.


####*How to run the code*
  >Start roscore on one terminal.
  >Run /auv/catkin_ws/scr/automate_dry_test/src/scripts/sensors_dry_test



###*Thrusters Dry test*

Before running the drytest, make sure to kill the planner, taskr and controls otherwise they will interfere
The dry test will cycle through each thruster and wait for input from the user to ensure functionality

During the dry test:

 >You will need to press enter to run each thruster
 
 >If a thruster is functional, you have the option to continue or test it again
 
####*How to run the code*
  >Start roscore on one terminal.
  >Run /auv/catkin_ws/scr/automate_dry_test/scripts/drytest_thruster
