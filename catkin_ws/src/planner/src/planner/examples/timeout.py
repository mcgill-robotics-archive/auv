#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-
import smach
import smach_ros
import rospy

__author__ = 'Max Krogius'


# This is an example of two states running in parallel, one of which
# will preempt the other after a certain amount of time.
class Loop(smach.State):
    def __init__(self, outcomes=['preempted', 'try_again']):
        super(Loop, self).__init__(outcomes=outcomes)

    def execute(self, userdata):
        while not self.preempt_requested():
            pass
        return 'preempted'


def create_state_machine():
    csm = smach.Concurrence(
        ['success', 'timer_failure', 'preempted'],
        default_outcome='timer_failure',
        # Preempt all children after first one finishes
        child_termination_cb=lambda x: True,
        outcome_map={
            'success': {'Timer': 'false'},
            'timer_failure': {'Timer': 'true'},
            'preempted': {'Timer': 'preempted'}})
    with csm:
        timer = smach_ros.ConditionState(
            lambda x: False,
            timeout=rospy.Duration(10.0),
            max_checks=-1)
        csm.add('Timer', timer)
        csm.add('Compliant Loop', Loop())
    return csm


if __name__ == '__main__':
    rospy.init_node('test_smach')
    csm = create_state_machine()
    sis = smach_ros.IntrospectionServer('test_smach', csm, '/SM_ROOT')
    sis.start()
    csm.execute()
    rospy.spin()
    sis.stop()
