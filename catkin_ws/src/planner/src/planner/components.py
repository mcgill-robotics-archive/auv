import rospy
import smach
from primitives import SetVelocityState


def switch_entry_point(state_machine, label):
    # Returns a state which switches state_machine's entry point to the
    # specified label.
    state = smach.State(['succeeded'])

    def execute(user_data):
        state_machine.set_initial_state([label])
        return 'succeeded'
    state.execute = execute
    return state


def search_pattern():
    # For now just come to a stop and look for 15 seconds.
    # TODO: Do fancier search pattern
    return SetVelocityState.create_move_forward_state(0, rospy.Duration(15))
