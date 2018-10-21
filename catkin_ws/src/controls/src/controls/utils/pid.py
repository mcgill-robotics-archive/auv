import rospy

trans_gains = rospy.get_param('PID/translation_gains')
rot_gains = rospy.get_param('PID/rotation_gains')


class PID(object):
    """
    Discrete PID control
    """
    def __init__(self,
                 kp,
                 ki,
                 kd,
                 differentiator=0,
                 integrator=0,
                 integrator_max=100,
                 integrator_min=-100):

        self.output = 0

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.differentiator = differentiator
        self.integrator = integrator
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min
        self.last_error = None

    def update(self, error, delta_time=0.1):
        """
        Calculate PID output value for given error
        """
        if error is None:
            return self.output

        # constant multiplier determined experimentally

        self.p_value = self.kp * error

        if self.last_error is None:
            self.d_value = 0
        else:
            derivative = (error - self.last_error) / delta_time
            self.d_value = self.kd * derivative

        self.last_error = error

        self.integrator = self.integrator + error * delta_time

        # Upper and lower bounds on the integrator help to mitigate wind up
        if self.integrator > self.integrator_max:
            self.integrator = self.integrator_max
        elif self.integrator < self.integrator_min:
            self.integrator = self.integrator_min

        self.i_value = self.integrator * self.ki

        self.output = self.p_value + self.i_value + self.d_value

        return self.output

    def reset(self):
        self.differentiator = 0
        self.integrator = 0
        self.last_error = None
