#!/usr/bin/env python

import rospy

from auv_msgs.msg import TaskLightActionGoal, TaskLightActionResult, TaskLightActionFeedback
from status.srv import TaskState
from blinky.msg import *
from blinky.srv import *

from time import sleep


"""
    arguments:
      req: request
      res: response

"""

# Blinky color selection
# I am going to add more colors, but I first need to test it
RED = RGB(255, 0, 0)
GREEN = RGB(0, 255, 0)
BLUE = RBG(0, 0, 255)
LIGHTBLUE = RBG(0, 0, 150)

YELLOW = RBG(180, 180, 0)

PURPLE = RGB(180, 0, 180)
PINK = RGB(100, 0, 0)
MAGENTA = RGB(200, 0, 100)

BACK = RGB(0, 0, 0)
WHITE = RGB(255, 255, 255)


def task_blinky_design(req):
    colors = []   # Reset the array
    task = req.status.status

    # Gate
    if(task == 'gate'):
        send_color(PINK)
        sleep(0.5)
        send_color(MAGENTA)
        sleep(0.5)
        send_color(RED)

    # Buoys
    elif(task == 'red_buoy'):
        send_color(RED)

    elif(task == 'green_buoy'):
        send_color(GREEN)

    elif(task == 'yellow_buoy'):
        send_color(YELLOW)

    # Maneuver
    elif(task == 'maneuver'):
        send_color(LIGHTBLUE)
        sleep(0.5)
        send_color(BLUE)

    # Bins
    elif(task == 'bins'):
        send_color()

    # Torpedo
    elif(task == 'torpedo'):
        send_color()

    # Octogon
    elif(task == 'octogon'):
        send_color()

    # Square
    elif(task == 'square'):
        send_color()

    else:
        rospy.logwarn("Unknown result status code : {0}!".format(result))
        send_color(BLACK)

    return TaskStateResponse(req)


def send_color(color):
    colors = []
    for i in range(25):
        colors.append(color)
    send_colors(colors)


def send_colors(colors):
    """Call Service in blinky_server

    req: RGB[] colors
    res: int8 success

    """
    try:
        blinky_proxy = rospy.ServiceProxy('update_planner_lights',
                                          UpdatePlannerLights)
        res = blinky_proxy(colors)
        if res.success != 0:
            print "UpdatePlannerLights request unsuccessful: %s"%res

    except Exception as e:
        print "Exception: %s"%e


def task_state_listener():
  """Create Service Taskr.py Calls

  req: String corresponding to the name of the task. Received by
            task_blinky_design

  res: String

  """
	rospy.init_node('task_state_listener')
	task_state = rospy.Service('task_state', TaskState, task_blinky_design)
	rospy.spin()


if __name__ == "__main__":
	task_state_listener()

