#!/usr/bin/env python

import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, PointCloud

from console_format import format
from wait_for_message import *


def check_depth_sensor():
    answer = 'y'
    skip = 'x'

    print('\n' + format.UNDERLINE + format.OKBLUE + 'Depth Sensor' +
          format.ENDC)

    skip = raw_input('About to test sensor, make sure it is launched, and '
                     'press most keys to continue (s to skip): ')

    if (skip.lower() == 's'):
        print(format.WARNING + 'Skipped...' + format.ENDC)
        return True

    while (answer.lower() == 'y'):
        isResponsive = True
        print('Waiting for sensor feedback...')

        # Test Raw Output of Depth Sensor
        try:
            wait_for_message('/raw_depth', Float64, 3)

        except Exception:
            print (format.WARNING + 'Raw feedback is unresponsive' +
                   format.ENDC)
            isResponsive = False
            pass

        # Test Pressure Output of Depth Sensor
        try:
            wait_for_message('/depth/pressure', Float64, 3)

        except Exception:
            print (format.WARNING + 'Pressure feedback is unresponsive' +
                   format.ENDC)
            isResponsive = False
            pass

        if isResponsive:
            answer = 'n'
        else:
            answer = raw_input('One or more of the messages was not being '
                               'published.\nPress y to try again and any '
                               'other key to move on to the next sensor: ')

    if isResponsive:
        print (format.OKGREEN + format.BOLD + 'The sensor is working!' +
               format.ENDC)
    else:
        print (format.FAIL + format.BOLD + 'The sensor is not responsive' +
               format.ENDC)

    return isResponsive


def check_imu():
    answer = 'y'
    skip = 'x'

    print('\n' + format.UNDERLINE + format.OKBLUE + 'IMU' +
          format.ENDC)

    skip = raw_input('About to test sensor, make sure it is launched, and '
                     'press most keys to continue (s to skip): ')

    if (skip.lower() == 's'):
        print(format.WARNING + 'Skipped...' + format.ENDC)
        return True

    while (answer.lower() == 'y'):
        isResponsive = True
        print('Waiting for sensor feedback...')

        # Test Raw Output of the IMU
        try:
            wait_for_message('/state_estimation/raw', Imu, 3)

        except Exception:
            print (format.WARNING + 'Raw feedback is unresponsive' +
                   format.ENDC)
            isResponsive = False
            pass

        if isResponsive:
            answer = 'n'
        else:
            answer = raw_input('One or more of the messages was not being '
                               'published.\nPress y to try again and any '
                               'other key to move on to the next sensor: ')

    if isResponsive:
        print (format.OKGREEN + format.BOLD + 'The sensor is working!' +
               format.ENDC)
    else:
        print (format.FAIL + format.BOLD + 'The sensor is not responsive' +
               format.ENDC)

    return isResponsive


def check_sonar():
    answer = 'y'
    skip = 'x'

    print('\n' + format.UNDERLINE + format.OKBLUE + 'Sonar' +
          format.ENDC)

    skip = raw_input('About to test sensor, make sure it is launched, and '
                     'press most keys to continue (s to skip): ')

    if (skip.lower() == 's'):
        print(format.WARNING + 'Skipped...' + format.ENDC)
        return True

    while (answer.lower() == 'y'):
        isResponsive = True
        print('Waiting for sensor feedback...')

        # Test Raw Output of the Sonar
        try:
            wait_for_message('/tritech_micron/scan', PointCloud, 3)

        except Exception:
            print (format.WARNING + 'Raw feedback on tritech_micron is '
                   'unresponsive' + format.ENDC)
            isResponsive = False
            pass

        # Test Full Scan Output from Sonar Proc
        try:
            wait_for_message('/sonar_proc/full_scan', PointCloud, 3)

        except Exception:
            print (format.WARNING + 'Full scan on sonar_proc is unresponsive' +
                   format.ENDC)
            isResponsive = False
            pass

        if isResponsive:
            answer = 'n'
        else:
            answer = raw_input('One or more of the messages was not being '
                               'published.\nPress y to try again and any '
                               'other key to move on to the next sensor: ')

    if isResponsive:
        print (format.OKGREEN + format.BOLD + 'The sensor is working!' +
               format.ENDC)
    else:
        print (format.FAIL + format.BOLD + 'The sensor is not responsive' +
               format.ENDC)

    return isResponsive


def run_test():
    functional = []
    non_functional = []

    print (format.OKBLUE + format.BOLD + '\n\n'
           ' #####################\n'
           ' ## TESTING SENSORS ##\n'
           ' #####################\n' + format.ENDC)

    # DEPTH SENSOR ------------------------------------------------------------
    if check_depth_sensor():
        functional.append('Depth Sensor')
    else:
        non_functional.append('Depth Sensor')

    time.sleep(1)

    # IMU ---------------------------------------------------------------------
    if check_imu():
        functional.append('IMU')
    else:
        non_functional.append('IMU')

    time.sleep(1)

    # SONAR -------------------------------------------------------------------
    if check_sonar():
        functional.append('Sonar')
    else:
        non_functional.append('Sonar')

    time.sleep(1)

    # SUMMARY -----------------------------------------------------------------
    print('\n' + format.UNDERLINE + format.OKBLUE + 'Summary' +
          format.ENDC)

    # Prints All Functional Sensors
    print ('\n' + format.OKGREEN + format.BOLD +
           'Functional (or skipped) Sensors are: ')
    for sensor in functional:
        print('  - ' + sensor)
    print (format.ENDC)

    # Prints All Functional Sensors
    print ('\n' + format.FAIL + format.BOLD + 'Non-functional Sensors are: ')
    for sensor in non_functional:
        print('  - ' + sensor)
    print (format.ENDC)

    print (format.OKBLUE + '\nFinished testing sensors\n\n' + format.ENDC)

    if (len(non_functional) > 0):
        return True  # >> isError
    else:
        return False  # >> is Error


if __name__ == "__main__":
    rospy.init_node("drytest_sensors")
    run_test()
