import rospy
import smach
import smach_ros

from tasks import *


class Initialize(smach.State):
	#This starts up the state machine! Just prints out a message
	#Then succeeds
	def __init__(self, task_topic, timeout):
		outcomes        = ['succeeded', 'timed_out', 'preempted']
		self.task_topic = task_topic
		self.timeout    = timeout
		smach.State.__init__(self, outcomes=outcomes)
	def execute(self, userdata):
		rospy.loginfo('[Global] Executing initialize state')
		return 'succeeded'


class ChooseTask(smach.State):
	def __init__(self, target_tasks, choose_task_rate):
		outcomes = ['gate',
					'square',
					'succeeded',
					'failed',
					'preempted']
		smach.State.__init__(self, outcomes=outcomes)


		self.target_tasks = target_tasks		
		self.rate         = rospy.Rate(choose_task_rate)

		self.task         = None
		self.timed_out    = False

	def execute(self, userdata):
		rospy.loginfo('[Global] Executing choose task state')
		return 'gate'
		


class Planner(object):
	def __init__(self, task_topic, attempted_tasks, init_timeout,
		         choose_task_rate, give_up_rate):
				
		rospy.loginfo('{}'.format(attempted_tasks))

		outcomes = ['succeeded', 'failed', 'preempted']
		self.sm  = smach.StateMachine(outcomes=outcomes)

		self.initialize        = Initialize(task_topic, init_timeout)
		self.gate              = Gate().get_state_machine()
		self.square            = Square().get_state_machine()
		self.choose_task       = ChooseTask(attempted_tasks, choose_task_rate)

		#self.task_sub          = rospy.Subscriber(task_topic, Task, self.choose_task.task_cb)

		with self.sm:
			smach.StateMachine.add(
				'INITIALIZE',
				self.initialize,
				{'succeeded':'CHOOSE_TASK',
				'timed_out' :'failed',
				'preempted' :'preempted'})
			smach.StateMachine.add(
				'GATE',
				self.gate,
				{'succeeded':'CHOOSE_TASK',
				'failed':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'SQUARE',
				self.square,
				{'succeeded':'CHOOSE_TASK',
				'failed':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'CHOOSE_TASK',
				self.choose_task,
				{'gate'    :'GATE',
				'square'   :'SQUARE',
				'succeeded':'succeeded',
				'failed'   :'failed',
				'preempted':'preempted'})

	def start(self):
		outcome = self.sm.execute()
		rospy.loginfo('State Machine exited. Result: {}'.format(outcome))
