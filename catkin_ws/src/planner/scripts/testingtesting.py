#!/usr/bin/python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty
from threading import Thread

class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])
    def execute(self,userdata):
        rospy.loginfo('initializing')
        rospy.sleep(10)
        if self.preempt_requested:
            #self.service_preempt()
            return 'succeeded'
        return 'succeeded'

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','preempted','notdone'],
                             input_keys=['uid'],
                             output_keys=['uid'])
    def execute(self,userdata):
        rospy.loginfo('Moving')
        rospy.sleep(10)
        #if self.preempt_requested:
            #self.service_preempt()
            #return 'succeeded'
        if userdata.uid==4:
            userdata.uid = 0
            return 'done'
        else:
            userdata.uid+=1
            return 'notdone'
        


def instastopper1(outcome_map):
    if outcome_map['Monitor'] == 'invalid' or outcome_map['Initialize']=='succeeded' :
        return True
    return False

def instastopper2(outcome_map):
    if outcome_map['Monitor']=='invalid' or outcome_map['Move']=='done' or outcome_map['Move']=='notdone':
        return True
    return False

def out_idle_cb(outcome_map):
    rospy.sleep(3.5)
    if outcome_map['Monitor']=='invalid':
        return 'invalid'
    else:
        return 'valid'

def out_cb(outcome_map):
    rospy.sleep(3.5)
    if outcome_map['Monitor']=='invalid':
        return 'stop'
    elif outcome_map['Initialize']=='succeeded':
        return 'continue'
    else:
        return 'stop'
    
    
def out_move_cb(outcome_map):
    rospy.sleep(3.5)
    if outcome_map['Monitor'] == 'invalid' or outcome_map['Move']=='done':
        return 'stop'
    elif outcome_map['Move']=='notdone':
        return 'continue'
    else:
        return 'stop'
    
    

def monitor_cb(ud, msg):
    rospy.loginfo('acting')
    return False

#def monitor_start_cb(ud,msg):
#    return False

def create_machine():

    
    init_concurrence = smach.Concurrence(outcomes=['stop','continue'],
                                           default_outcome='continue',
                                           child_termination_cb=instastopper1,
                                           outcome_cb=out_cb)
    with init_concurrence:
        
        smach.Concurrence.add('Initialize',
                               Initialize())

       

        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb))


    static_sm = smach.StateMachine(outcomes=['done'])

    move_concurrence = smach.Concurrence(outcomes=['stop','continue'],
                                           default_outcome='continue',
                                           input_keys=['uid'],
                                           output_keys=['uid'],
                                           child_termination_cb=instastopper2,
                                           outcome_cb=out_move_cb)
    with move_concurrence:
        smach.Concurrence.add('Move',
                               Move())

       
        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb))


    idle_concurrence=smach.Concurrence(outcomes=['valid','invalid'],
                                       default_outcome='valid',
                                       outcome_cb=out_idle_cb)

    with idle_concurrence:
        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb)) 

        
    
    with static_sm:
        #countdown = rospy.get_param('countdown')
        #countdown1 = rospy.Time(10)
        static_sm.userdata.uid = 0 
        smach.StateMachine.add('Idle',
                               idle_concurrence,
                               transitions={'invalid':'Init',
                                            'valid':'Idle'})
        smach.StateMachine.add('Init', init_concurrence, transitions={'continue':'Movement',
                                                                      'stop':'Idle'})
        smach.StateMachine.add('Movement', move_concurrence, transitions={'continue':'Movement',
                                                                          'stop':'Idle'},
                               remapping={'uid':'uid'})

    return static_sm
                               
                                               
        
        


























if __name__ == '__main__':
    rospy.init_node('square_state_machine',log_level=rospy.DEBUG)
    
##    server1 = InitializeServer('init_action')
##    server2 = move_server.MoveAction('square_action')
    
    sm = create_machine()
 
    sis = smach_ros.IntrospectionServer('square_state_machine', sm, '/SM_ROOT')
    sis.start  #taken from the threading method used in auv-2015 to avoid problems
    smach_thread = Thread(target=lambda: sm.execute())
    smach_thread.start()

    #taken from auv-2015 to avoid the ctrl+c issue
    rospy.on_shutdown(sm.request_preempt)
    #sm.execute()
    rospy.spin()
    
    
    
    smach_thread.join()
    sis.stop()
