
from geometry_msgs.msg import *
import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
import random

class InitializeServer(object):
    _feedback = initializeFeedback()
    _result = initializeResult()



    def __init__(self, name):
        
        
        self._as = SimpleActionServer(name, initializeAction, execute_cb=self.execute_cb, auto_start=False)
        
        
        self._as.start()

    def execute_cb(self,goal):
        rospy.loginfo('starting')
        t = goal
        rospy.loginfo(goal)
        seconds = int(goal.countdown.secs) #floating point
        nanoseconds = goal.countdown.nsecs

        for s in range(seconds,0,-1):
             if self._as.is_preempt_requested():
                rospy.loginfo('Initialize Preempted')
                self._as.set_preempted()
                success = False
                break
             #rand = random.randint(0,1)
            
                
             self._feedback.time_left = rospy.Time.from_sec(s)
             self._as.publish_feedback(self._feedback)
             rospy.loginfo(self._feedback)
             #if rand == 0:
             
             #break
             rospy.Rate(1).sleep()

        self._result.offset = Pose(Point(10,10,10),Quaternion(20,20,20,20))
        rospy.loginfo('Initialize Succeeded')
        self._as.set_succeeded(self._result)




##        rospy.loginfo('Initialize Preempted')
##        self._as.set_preempted() 
            

            
        
