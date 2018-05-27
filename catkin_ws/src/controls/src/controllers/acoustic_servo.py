import rospy
from std_msgs.msg import Float64

from utils import normalize_angle
from servo_controller import YawMaintainer


class AcousticServoController(YawMaintainer):
    '''
    AcousticServo provides a controller to minimize the error between the
    robot's current heading and the heading of the pinger
    '''
    def __init__(self):
        super(AcousticServoController, self).__init__()
        self.sub = None
        self.is_pinger_heard = False
        self.heading = None
        self.last_heading_time = None

    def start(self):
        super(AcousticServoController, self).start()
        self.sub = rospy.Subscriber('/hydrophones/heading', Float64,
                                    self._update_pinger_heading)

    def stop(self):
        super(AcousticServoController, self).stop()
        if self.sub is not None:
            self.sub.unregister()

    def _update_pinger_heading(self, msg):
        self.is_pinger_heard = True
        self.heading = msg.data
        self.last_heading_time = rospy.Time.now()
        current_yaw = self.get_current_yaw()
        self.set_setpoint(normalize_angle(current_yaw + msg.data))

    def heard_pinger(self):
        return self.is_pinger_heard
