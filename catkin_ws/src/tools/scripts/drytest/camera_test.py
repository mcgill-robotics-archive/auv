#!/usr/bin/env python

import time
import subprocess
from sensor_msgs.msg import Image

from console_format import format
from wait_for_message import *


def check_front_camera():
    cont = 'y'  # Continue the camera test loop (feedback)
    skip = 'x'  # Skip this camera (feedback)
    viewFb = 'n'  # Feedback as to whether the video is good or not

    # Command to open the image feed
    cmd = '\"rosrun image_view image_view image:=/camera_front/image_color\"'

    print('\n' + format.UNDERLINE + format.OKBLUE + 'Front Camera' +
          format.ENDC)

    skip = raw_input('About to test sensor, make sure it is launched, and '
                     'press most keys to continue (s to skip): ')

    if (skip.lower() == 's'):
        print(format.WARNING + 'Skipped...' + format.ENDC)
        return True

    while (cont.lower() == 'y'):
        isResponsive = True
        isGoodView = False

        print('Waiting for sensor feedback...')

        # Make sure that the node is actually publishing
        try:
            wait_for_message('/camera_front/image_color', Image, 3)

        except Exception:
            print (format.WARNING + 'Raw feedback is unresponsive' +
                   format.ENDC)
            isResponsive = False
            pass

        if isResponsive:
            raw_input('The camera node is responsive.\nThe test will now '
                      'open a new terminal and will only resume ' +
                      format.BOLD + format.WARNING +
                      'COMPLETELY EXIT THAT WINDOW\n' +
                      format.ENDC + 'Press any key to continue ')

            # A new subprocess opens another terminal and runs image_view
            # The main test process will be blocked until this subprcoess
            # is killed. Could not get this to run in the same process in
            # sequence because image_view doesn't like to exit cleanly...
            try:
                subprocess.call('xterm -e ' + cmd, shell=True)
            except Exception:
                print(format.FAIL + 'Could not open a xterm' +
                      format.ENDC)

            viewFb = raw_input('Is this feed good? y/n: ')

            if (viewFb.lower() == 'y'):
                isGoodView = True
                break

        if ((not isResponsive) or (not isGoodView)):
            cont = raw_input('The camera is unresponsive or the view is not '
                             'good.\nPress y to try again and any '
                             'other key to move on to the next sensor: ')

    if isGoodView:
        print (format.OKGREEN + format.BOLD + 'The camera is working!' +
               format.ENDC)
    else:
        print (format.FAIL + format.BOLD + 'The camera is not working' +
               format.ENDC)

    return isGoodView


def check_down_camera():
    cont = 'y'  # Continue the camera test loop (feedback)
    skip = 'x'  # Skip this camera (feedback)
    viewFb = 'n'  # Feedback as to whether the video is good or not

    # Command to open the image feed
    cmd = '\"rosrun image_view image_view image:=/camera_down/image_color\"'

    print('\n' + format.UNDERLINE + format.OKBLUE + 'Down Camera' +
          format.ENDC)

    skip = raw_input('About to test sensor, make sure it is launched, and '
                     'press most keys to continue (s to skip): ')

    if (skip.lower() == 's'):
        print(format.WARNING + 'Skipped...' + format.ENDC)
        return True

    while (cont.lower() == 'y'):
        isResponsive = True
        isGoodView = False

        print('Waiting for sensor feedback...')

        # Make sure that the node is actually publishing
        try:
            wait_for_message('/camera_down/image_color', Image, 3)

        except Exception:
            print (format.WARNING + 'Raw feedback is unresponsive' +
                   format.ENDC)
            isResponsive = False
            pass

        if isResponsive:
            raw_input('The camera node is responsive.\nThe test will now '
                      'open a new terminal and will only resume ' +
                      format.BOLD + format.WARNING +
                      'COMPLETELY EXIT THAT WINDOW\n' +
                      format.ENDC + 'Press any key to continue ')

            # A new subprocess opens another terminal and runs image_view
            # The main test process will be blocked until this subprcoess
            # is killed. Could not get this to run in the same process in
            # sequence because image_view doesn't like to exit cleanly...
            try:
                subprocess.call('xterm -e ' + cmd, shell=True)
            except Exception:
                print(format.FAIL + 'Could not open a xterm' +
                      format.ENDC)

            viewFb = raw_input('Is this feed good? y/n: ')

            if (viewFb.lower() == 'y'):
                isGoodView = True
                break

        if ((not isResponsive) or (not isGoodView)):
            cont = raw_input('The camera is unresponsive or the view is not '
                             'good.\nPress y to try again and any '
                             'other key to move on to the next sensor: ')

    if isGoodView:
        print (format.OKGREEN + format.BOLD + 'The camera is working!' +
               format.ENDC)
    else:
        print (format.FAIL + format.BOLD + 'The camera is not working' +
               format.ENDC)

    return isGoodView


def run_test():
    functional = []
    non_functional = []

    print (format.OKBLUE + format.BOLD + '\n\n'
           ' #####################\n'
           ' ## TESTING CAMERAS ##\n'
           ' #####################\n' + format.ENDC)

    # FRONT CAMERA ------------------------------------------------------------
    if check_front_camera():
        functional.append('Front Camera')
    else:
        non_functional.append('Front Camera')

    time.sleep(1)

    # DOWN CAMERA -------------------------------------------------------------
    if check_down_camera():
        functional.append('Down Camera')
    else:
        non_functional.append('Down Camera')

    time.sleep(1)

    # SUMMARY -----------------------------------------------------------------
    print('\n' + format.UNDERLINE + format.OKBLUE + 'Summary' +
          format.ENDC)

    # Prints All Functional Sensors
    print ('\n' + format.OKGREEN + format.BOLD +
           'Functional (or skipped) Cameras are: ')
    for camera in functional:
        print('  - ' + camera)
    print (format.ENDC)

    # Prints All Functional Sensors
    print ('\n' + format.FAIL + format.BOLD + 'Non-functional Cameras are: ')
    for camera in non_functional:
        print('  - ' + camera)
    print (format.ENDC)

    print (format.OKBLUE + '\nFinished testing cameras\n\n' + format.ENDC)

    if (len(non_functional) > 0):
        return True, non_functional  # >> isError
    else:
        return False, None  # >> is Error


if __name__ == "__main__":
    rospy.init_node("drytest_sensors")
    run_test()
