#!/usr/bin/env python

import rospy
import os

import camera_test
import thrusters_test
import sensors_test
from console_format import format

if __name__ == "__main__":
    rospy.init_node("drytest_node")

    isError = False
    runTest = 'n'

    os.system('clear')

    # INITIALIZE --------------------------------------------------------------
    print(format.GREY + format.BOLD + format.DIM +
          '\n>>>>>>>>>>>> Starting Drytest' + format.ENDC)

    print(format.RED + format.BOLD +
          '\n  #########################\n'
          '  ##                     ##\n'
          '  ## ' + format.WHITE + '  McGill Robotics  ' + format.RED + ' ##\n'
          '  ## ' + format.WHITE + '    AUV Drytest    ' + format.RED + ' ##\n'
          '  ##                     ##\n'
          '  #########################')

    # CHECK SENSORS -----------------------------------------------------------
    runTest = raw_input(format.WARNING + '\nTest sensors? (y/n): ' +
                        format.ENDC)

    if (runTest.lower() == 'y'):
        isError = (sensors_test.run_test() or isError)
    else:
        print (format.OKBLUE + 'Skipped sensor tests' + format.ENDC)

    # CHECK THRUSTERS ---------------------------------------------------------
    runTest = raw_input(format.WARNING + '\nTest thrusters? (y/n): ' +
                        format.ENDC)

    if (runTest.lower() == 'y'):
        isError = (thrusters_test.run_test() or isError)
    else:
        print (format.OKBLUE + 'Skipped thruster tests' + format.ENDC)

    # CHECK CAMERAS -----------------------------------------------------------
    runTest = raw_input(format.WARNING + '\nTest cameras? (y/n): ' +
                        format.ENDC)

    if (runTest.lower() == 'y'):
        isError = (camera_test.run_test() or isError)
    else:
        print (format.OKBLUE + 'Skipped camera tests' + format.ENDC)

    # CONCLUSION --------------------------------------------------------------
    if (not isError):
        print(format.BOLD + format.OKGREEN + '\n\n'
              '  ##############\n'
              '  #            #\n'
              '  #  Passing!  #\n'
              '  #            #\n'
              '  ##############\n' + format.ENDC)
    else:
        print(format.BOLD + format.FAIL + '\n\n'
              '  ##############\n'
              '  #            #\n'
              '  #    Fail    #\n'
              '  #            #\n'
              '  ##############\n' + format.ENDC)

    print(format.GREY + format.BOLD + format.DIM +
          '\n<<<<<<<<<<<< Finished Drytest\n' + format.ENDC)
