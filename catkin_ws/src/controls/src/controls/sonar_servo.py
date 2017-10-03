import rospy
from auv_msgs.msg import TaskPointsArray
import math
from utils import normalize_angle
from servo_controller import YawMaintainer


class SonarServoController(YawMaintainer):
    '''
    SonarServo provides a controller to minimize the error between the
    robot's current heading and the heading of the current TaskPoint
    '''
    def __init__(self):
        super(SonarServoController, self).__init__()
        self.sub = None

    def start(self):
        super(SonarServoController, self).start()
        self.sub = rospy.Subscriber("sonar_proc/task_point", TaskPointsArray,
                                    self._update_sonar_header)

    def stop(self):
        super(SonarServoController, self).stop()
        if self.sub is not None:
            self.sub.unregister()

    def _update_sonar_header(self, msg):
        self.is_TaskPoint = True
        current_yaw = self.get_current_yaw()
        yaw_diff = math.atan(msg.data.task.y / msg.data.task.x)
        self.heading = yaw_diff
        self.set_setpoint(normalize_angle(current_yaw + yaw_diff))

    def saw_TaskPoint(self):
        return self.is_TaskPoint
