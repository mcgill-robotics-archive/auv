#!/usr/bin/env python

import time
import subprocess
from sensor_msgs.msg import Image

from console_format import format
from wait_for_message import *


class CameraTest:

    def __init__(self):
        # Load cameras from the config file
        self.cameras = rospy.get_param('/drytest/cameras')

        # Delay between camera checks
        self.delay = 2

        # Command to open the image feed
        self.cmd = ('\"rosrun image_view image_view image:=')

        # Stores the results of the test
        self.results = {
            'passes': [],
            'fails': []
        }

    def check_camera(self, camera):
        cont = 'y'  # Continue the camera test loop (feedback)
        skip = 'x'  # Skip this camera (feedback)
        view_fb = 'n'  # Feedback as to whether the video is good or not

        print('\n' + format.UNDERLINE + format.OKBLUE + camera['name'] +
              format.ENDC)

        skip = raw_input('About to test the camera, make sure it is launched, '
                         'and enter most keys to continue [s to skip] ')

        if (skip.lower() == 's'):
            print(format.WARNING + 'Skipped...' + format.ENDC)
            return True

        while (cont.lower() == 'y'):
            is_responsive = True
            is_good_view = False

            print('Waiting for sensor feedback...')

            # Make sure that the node is actually publishing
            try:
                wait_for_message(camera['topic'], Image, 3)

            except Exception:
                print (format.WARNING + camera['topic'] + ' is unresponsive' +
                       format.ENDC)
                is_responsive = False
                pass

            if is_responsive:
                raw_input('The camera node is responsive.\nThe test will now '
                          'open a new terminal and will only resume ' +
                          format.BOLD + format.WARNING +
                          'COMPLETELY EXIT THAT WINDOW\n' +
                          format.ENDC + 'Enter any key to continue ')

                # A new subprocess opens another terminal and runs image_view
                # The main test process will be blocked until this subprcoess
                # is killed. Could not get this to run in the same process in
                # sequence because image_view doesn't like to exit cleanly...
                try:
                    view_call = self.cmd + ' image:=' + camera['topic'] + '\"'
                    subprocess.call('xterm -e ' + view_call, shell=True)
                except Exception as e:
                    print(format.FAIL + 'Could not open xterm' + format.ENDC)
                    print(e)
                    break

                view_fb = raw_input('Is this feed good? [y/N] ')

                if (view_fb.lower() == 'y'):
                    is_good_view = True
                    break

            if ((not is_responsive) or (not is_good_view)):
                cont = raw_input('The camera is unresponsive or the view is '
                                 'not good.\nTry again? [y/N] ')

        if is_good_view:
            self.results['passes'].append(camera['name'])
            print (format.OKGREEN + format.BOLD + 'The camera is working!' +
                   format.ENDC)
        else:
            self.results['fails'].append(camera['name'])
            print (format.FAIL + format.BOLD + 'The camera is not working' +
                   format.ENDC)

    def run_test(self):
        is_error = False

        print (format.OKBLUE + format.BOLD + '\n\n'
               ' #####################\n'
               ' ## TESTING CAMERAS ##\n'
               ' #####################\n' + format.ENDC)

        # TEST CAMERAS --------------------------------------------------------
        for camera in self.cameras:
            self.check_camera(camera)
            time.sleep(self.delay)

        # SUMMARY -------------------------------------------------------------
        print('\n' + format.UNDERLINE + format.OKBLUE + 'Summary' +
              format.ENDC)

        # Prints All Functional Cameras
        print (format.OKGREEN + format.BOLD + 'Functional Cameras are: ')
        for camera in self.results['passes']:
            print('- ' + camera)

        # Prints All Functional Sensors
        print (format.FAIL + format.BOLD + 'Non-functional Sensors are: ')
        for camera in self.results['fails']:
            print('- ' + camera)
            is_error = True
        print (format.ENDC)

        print (format.OKBLUE + 'Finished testing cameras\n' + format.ENDC)

        return is_error
