import rospy

from std_msgs.msg import Float64

from utils import AsyncServoController
from pid import Pid, trans_gains


class DepthMaintainer(AsyncServoController):
    def __init__(self, setpoint=None):
        pid = Pid(*trans_gains['heave'])

        pub = rospy.Publisher('controls/superimposer/heave',
                              Float64,
                              queue_size=1)
        self.error = None

        if setpoint is None:
            setpoint = rospy.wait_for_message('state_estimation/depth',
                                              Float64,
                                              timeout=5.0).data

        super(DepthMaintainer, self).__init__(
            pid, pub, 'state_estimation/depth', Float64, setpoint)

    def get_error(self, msg):
        self.error = self.setpoint - msg.data
        return self.error
