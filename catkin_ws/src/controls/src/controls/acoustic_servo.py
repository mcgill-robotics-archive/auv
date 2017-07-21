import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

from PID import PID, rot_gains
from utils import normalize_angle
from servo_controller import ASyncServoController


class AcousticServo(ASyncServoController):
    '''
    AcousticServo provides a controller to minimize the error between the
    robot's current heading and the heading of the pinger
    '''
    def __init__(self):
        pid = PID(*rot_gains['yaw'])
        pub = rospy.Publisher('controls/superimposer/yaw', PoseStamped,
                              queue_size=1)
        self.error = None

        super(YawMaintainer, self).__init__(pid, pub, 'hydrophones/heading',
                                            PoseStamped, 0)

    def get_error(self, msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)
        (_, _, yaw) = euler_from_quaternion(quaternion)

        normalized_yaw_error = normalize_angle(yaw)
        self.error = normalized_yaw_error
        return self.error
