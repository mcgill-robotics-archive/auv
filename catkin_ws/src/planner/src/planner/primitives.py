# -*- coding: utf-8 -*-

import rospy
import auv_msgs.msg
from smach import State
from smach_ros import SimpleActionState
from actionlib.simple_action_client import GoalStatus

__author__ = 'Max Krogius, Anass Al-Wohoush'


class FireDropperState(State):
    '''Fires a dropper'''
    def __init__(self):
        self.pub = rospy.Publisher(
            'electrical_interface/solenoid',
            auv_msgs.msg.SolenoidCommands, queue_size=100)
        super(FireDropperState, self).__init__(outcomes=['succeeded'])

    def execute(self, user_data):
        global pub
        self.num_pubs = 10
        rospy.Timer(rospy.Duration(0.1), self.fire_cb)
        return 'succeeded'

    def fire_cb(self, event):
        fire = self.num_pubs > 0
        self.num_pubs -= 1
        cmd = auv_msgs.msg.SolenoidCommands()
        cmd.port_dropper = fire
        cmd.starboard_dropper = fire
        cmd.port_torpedo = fire
        self.pub.publish(cmd)


class SetUserDataState(State):
    '''Outputs the given user data'''

    def __init__(self, user_data):
        self.user_data = user_data
        super(SetUserDataState, self).__init__(
            outcomes=['succeeded'],
            output_keys=user_data.keys())

    def execute(self, user_data):
        user_data.update(self.user_data)
        return 'succeeded'


class InitializationState(SimpleActionState):

    '''Initializes horizon to current heading.'''

    def __init__(self, countdown):
        '''
        Args:
            countdown: ROS Duration of initialization process.
        '''
        if not isinstance(countdown, rospy.Duration):
            countdown = rospy.Duration(countdown)

        # Set up goal.
        goal = auv_msgs.msg.InitializeHorizonGoal()
        goal.countdown = countdown

        super(InitializationState, self).__init__(
            'initialize_horizon',
            auv_msgs.msg.InitializeHorizonAction,
            goal=goal)


class SetVelocityState(SimpleActionState):

    '''
    A state to send open loop velocity commands to controls. Sends a
    SetVelocity command and exits after a given duration has passed,
    after which the SetVelocityAction is cancelled. To be used as a
    building block for more user friendly states.
    '''

    def __init__(self, goal_cb, duration=None, feedback_cb=None,
                 success_cb=None, input_keys=[], output_keys=[]):
        '''
        Args:
            goal_cb: Callback called when the state becomes active. Variables
                which need to be initialized each time the state starts should
                be initialized here. Will be called with the user_data as an
                argument. It should return a SetVelocity message.
            duration: How long to allow controls to work.
            feedback_cb: Callback to recieve feedback from the action server.
            success_cb: Called on success. Do cleanup here.
        '''
        self.goal_cb = goal_cb
        self.duration = duration
        self.feedback_cb = feedback_cb
        self.success_cb = success_cb
        super(SetVelocityState, self).__init__(
            'controls_velocity',
            auv_msgs.msg.SetVelocityAction,
            goal_cb=self._goal_cb,
            result_cb=self.result_cb,
            input_keys=input_keys,
            output_keys=output_keys)

    def _goal_cb(self, user_data, goal):
        # Initialize this state.
        self.succeeded = False
        self.initial_time = rospy.Time.now()

        # Get the goal to send to controls.
        self.goal_cb(user_data, goal)

    def _goal_feedback_cb(self, feedback):
        super(SetVelocityState, self)._goal_feedback_cb(feedback)

        # Exit if we timed out.
        if (self.duration is not None and
                rospy.Time.now() - self.initial_time > self.duration):
            self.exit_success()
        elif self.feedback_cb is not None:
            self.feedback_cb(self, feedback)

    def result_cb(self, user_data, goal_status, goal_result):
        # If we preempted based on the timeout, the result should be success.
        if self.succeeded and goal_status == GoalStatus.PREEMPTED:
            if self.success_cb is not None:
                self.success_cb(user_data)
            return 'succeeded'
        # For all other cases, we let the super handle it, so no need to
        # return anything.
        return None

    def exit_success(self):
        # This will cancel the pending action and transition to the next state.
        # The preempt is redefined to be success in result_cb.
        self.succeeded = True
        self.request_preempt()

    @classmethod
    def create_move_forward_state(cls, speed, duration):
        '''
        Creates a state which moves the robot forward at the given speed for
        the given duration. duration should be a rospy.Duration instance.
        '''
        def goal_cb(user_data, goal):
            goal.cmd.yaw = user_data.yaw_setpoint
            goal.cmd.surgeSpeed = speed
            goal.cmd.depth = user_data.depth_setpoint

        return cls(
            goal_cb,
            duration=duration,
            input_keys=['yaw_setpoint', 'depth_setpoint'],
            output_keys=['yaw_setpoint', 'depth_setpoint'])

    @classmethod
    def create_set_depth_state(cls, depth, tolerance=0.01):
        '''
        Creates a state which moves the robot to the given depth. The depth
        must be within the tolerance twice in a row before the state exits.
        '''
        def goal_cb(user_data, goal):
            goal.cmd.yaw = user_data.yaw_setpoint
            goal.cmd.depth = depth

        def feedback_cb(state, feedback):
            if abs(feedback.depth_error) < tolerance:
                state.exit_success()

        def success_cb(user_data):
            user_data.depth_setpoint = depth

        return cls(
            goal_cb,
            feedback_cb=feedback_cb,
            success_cb=success_cb,
            input_keys=['yaw_setpoint', 'depth_setpoint'],
            output_keys=['yaw_setpoint', 'depth_setpoint'])

    @classmethod
    def create_set_yaw_state(cls, yaw_offset, tolerance=0.01):
        '''
        Creates a state to set the yaw of the robot. Terminates when the yaw
        error is less than tolerance twice in a row.
        '''
        def goal_cb(user_data, goal):
            goal.cmd.yaw = user_data.yaw_setpoint + yaw_offset
            goal.cmd.depth = user_data.depth_setpoint

        def feedback_cb(state, feedback):
            if abs(feedback.yaw_error) < tolerance:
                state.exit_success()

        def success_cb(user_data):
            user_data.yaw_setpoint += yaw_offset

        return cls(
            goal_cb,
            feedback_cb=feedback_cb,
            success_cb=success_cb,
            input_keys=['yaw_setpoint', 'depth_setpoint'],
            output_keys=['yaw_setpoint', 'depth_setpoint'])


class SetPositionState(SimpleActionState):
    def __init__(self, goal_cb, pos_tol=0.05, yaw_tol=0.04):
        self.goal_cb = goal_cb
        self.pos_tol = pos_tol
        self.yaw_tol = yaw_tol
        super(SetPositionState, self).__init__(
            'controls_position',
            auv_msgs.msg.SetPositionAction,
            goal_cb=self._goal_cb,
            result_cb=self.result_cb,
            input_keys=['yaw_setpoint', 'depth_setpoint'],
            output_keys=['yaw_setpoint', 'depth_setpoint'])

    def _goal_cb(self, user_data, goal):
        # Initialize this state.
        self.succeeded = False
        # Get the goal to send to controls.
        self.goal_cb(user_data, goal)

    def _goal_feedback_cb(self, feedback):
        super(SetPositionState, self)._goal_feedback_cb(feedback)

        if (feedback.yaw_error < self.yaw_tol and
                feedback.x_error**2 + feedback.y_error**2 < self.pos_tol**2):
            self.exit_success()

    def result_cb(self, user_data, goal_status, goal_result):
        # If we preempted based on the timeout, the result should be success.
        if self.succeeded and goal_status == GoalStatus.PREEMPTED:
            return 'succeeded'
        # For all other cases, we let the super handle it, so no need to
        # return anything.
        return None

    def exit_success(self):
        # This will cancel the pending action and transition to the next state.
        # The preempt is redefined to be success in result_cb.
        self.succeeded = True
        self.request_preempt()


class AcousticServoState(SimpleActionState):
    def __init__(self, tdoa_tol=0.05):
        self.tdoa_tol = tdoa_tol
        super(AcousticServoState, self).__init__(
            'controls_hydro',
            auv_msgs.msg.AcousticServoAction,
            goal_cb=self._goal_cb,
            result_cb=self.result_cb,
            input_keys=['yaw_setpoint', 'depth_setpoint'],
            output_keys=['yaw_setpoint', 'depth_setpoint'])

    def _goal_cb(self, user_data, goal):
        # Initialize this state.
        self.succeeded = False
        # Get the goal to send to controls.
        goal.cmd.depth = user_data['depth_setpoint']
        goal.cmd.yaw = user_data['yaw_setpoint']

    def _goal_feedback_cb(self, feedback):
        super(SetPositionState, self)._goal_feedback_cb(feedback)

        if feedback.x_error**2 + feedback.y_error**2 < self.tdoa_tol**2:
            self.exit_success()

    def result_cb(self, user_data, goal_status, goal_result):
        # If we preempted based on the timeout, the result should be success.
        if self.succeeded and goal_status == GoalStatus.PREEMPTED:
            return 'succeeded'
        # For all other cases, we let the super handle it, so no need to
        # return anything.
        return None

    def exit_success(self):
        # This will cancel the pending action and transition to the next state.
        # The preempt is redefined to be success in result_cb.
        self.succeeded = True
        self.request_preempt()
