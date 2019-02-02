#! /usr/bin/env python

import rospy

import actionlib

import depth.msg
from PID import PID, trans_gains, rot_gains

class depthAction(object):
    # create messages that are used to publish feedback/result
    _feedback = depth.msg.fibsFeedback()
    _result = fibs.msg.fibsResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, depth.msg.depthAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()     

	#define the PID to be used in the Callback
	pid_depth = PID(*trans_gains['heave'])
	
	#define the channel that we need to publish to in order to send commands to the thrusters. The PID results will go here
	pub_depth = rospy.Publisher('controls/superimposer/heave', Float64,queue_size=1)
	
    def execute_cb(self, goal,pid_depth,pub_depth):
		'''
		The callback for this server is to update the PID. 
		This continues forever, unless given a new goal or pre-empted by the Mission switch. 
		I think I'm going to implement the mission switch as another client that can pre-empt this server
		(or I could be hacky- and listen to the mission topic on each loop)
		'''
		
        print('got here callback')
        # helper variables
        r = rospy.Rate(1)
        success = True
        print('rates Defined')
		self.setpoint=depth.msg.goal
		print('setpoint updated. setpoint = %f' % (self.setpoint))
		
        # start executing the action
        while True:
			print('in the while loop')
            # check that preempt has not been requested by the client
            
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                #Set PID Parameters to zero, and get out of this loop
                self.pid_depth.reset()
                break
            
            #Sloppy way of listening to the mission switch for now
            #When the mission topic returns 0, get out of the loop
            #DOUBLE CHECK WITH JEREMY IF I"VE DONE THIS RIGHT BEFORE RUNNING
            MISSION_TOPIC = rospy.get_param("planner/mission_topic")
            if MISSON_TOPIC ==0:
				break
				
            #Update the PID
				#First, get the depth, I'm at.This might be a bit hacky
            self.currentdepth = rospy.wait_for_message('state_estimation/depth',Float64, timeout=5.0).data)
            self.error = self.setpoint-self.currentdepth  
                #Next, use this to update the PID     
            self.result = self.pid_depth.update(self.error, last_duration) 
				#Thirdly, publish the error to the superimposer, which turns it into a command to the thrusters	
			self.pub_depth.publish(self.result)                                     
                # Finally, publish info to the console for the user
            rospy.loginfo('%s: Executing, sending depth of %i to the PID, current error %i' % (self._action_name, self.setpoint, self.result)
			print('PID updaated onceeeeee')
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
   
        #this will never succeed (?)
        #if success:
        #    self._result.sequence = self._feedback.sequence
        #    rospy.loginfo('%s: Succeeded' % self._action_name)
        #    self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('depth_server')
    server = depthAction(rospy.get_name())
    rospy.spin()
