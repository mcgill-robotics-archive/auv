

import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
from geometry_msgs.msg import *

class MoveAction(object):
   #setup the move server

   _feedback = moveFeedback()
   _result   = moveResult()
   def __init__(self, name):
    self._action_name = name
    self._as = SimpleActionServer(self._action_name, moveAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

   def execute_cb(self,goal):
      #grab all the parameters

       success = True
       ti = int(goal.time.secs)
       ve = goal.velocity
       th = goal.theta
       de = goal.depth

      #looping over the amount of time
       for s in range(ti,1,-1):
             if self._as.is_preempt_requested():
                #checking for preempts
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            #random move calculations TODO: calculations that actually make sense
             #de = de-ve.linear.z
             #th = th-ve.angular.z
             self._feedback.time_left = rospy.Time().from_sec(ti)
             self._feedback.depth = de
             self._feedback.theta = th
             self._as.publish_feedback(self._feedback)
             rospy.loginfo(self._feedback)
             rospy.Rate(1).sleep

      #what to send after succesfully moving

       if success:
           self._result.success = True
           rospy.loginfo('%s: Succeeded' % self._action_name)
           self._as.set_succeeded(self._result)
