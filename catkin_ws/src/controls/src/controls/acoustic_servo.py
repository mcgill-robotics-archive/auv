import rospy
from std_msgs.msg import Float64

from utils import normalize_angle
from servo_controller import YawMaintainer


class AcousticServo(YawMaintainer):
    '''
    AcousticServo provides a controller to minimize the error between the
    robot's current heading and the heading of the pinger
    '''
    def __init__(self):
        super(AcousticServo, self).__init__()

    def start(self):
        super(AcousticServo, self).start()
        self.sub = rospy.Subscriber('/hydrophones/heading', Float64,
            self._update_pinger_header)

    def stop(self):
        super(AcousticServo, self).stop()
        self.sub.unregister()

    def _update_pinger_header(self, msg):
        current_yaw = self.get_current_yaw()
        self.setpoint = normalize_angle(current_yaw + msg.data)
