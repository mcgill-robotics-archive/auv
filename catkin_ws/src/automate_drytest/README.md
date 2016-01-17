###**AUTOMATE DRYTEST**

##**Test**

#*Sensors Dry Test*

*sensors_dry_test.py* together with *wait_for_for message.py* detect whether or not given sensors are connected to the robot.


*sensors_dry_test.py* imports the *wait_for_for message.py* classes library. It then defines a *Testing* node and call the function wait_for_message inside *wait_for_message.py* mentioning the topic that is being tested.


*wait_for_for message.py* defines the node as a Subscriber and subsribes to the topic. It detect whether or not the selected sensor is connected to the robot.

A message is returned to *sensors_dry_test.py*.
>If the sensor is not detected: Returns "None"
>If the sensor is detected: Returns data published by the sensor inside the topic.

Each connection is done within a finite time thanks to a timer. The value of Timout can be modified.