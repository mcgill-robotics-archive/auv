import rospy

trans_gains = rospy.get_param('PID/translation_gains')
rot_gains = rospy.get_param('PID/rotation_gains')


class PID(object):
    """
    Discrete PID control
    """
    def __init__(self, P, I, D, Differentiator=0, Integrator=0,
                 Integrator_max=100, Integrator_min=-100):
        self.output = 0

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Differentiator = Differentiator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.last_error = None

    def update(self, error, delta_time=0.1):
        """
        Calculate PID output value for given error
        """
        if error is None:
            return self.output

        # constant multiplier determined experimentally

        self.P_value = self.Kp * error

        if self.last_error is None:
            self.D_value = 0
        else:
            derivative = (error - self.last_error) / delta_time
            self.D_value = self.Kd * derivative

        self.last_error = error

        self.Integrator = self.Integrator + error * delta_time

        # Upper and lower bounds on the integrator help to mitigate wind up
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        self.output = self.P_value + self.I_value + self.D_value

        return self.output

    def reset(self):
        self.Differentiator = 0
        self.Integrator = 0
        self.last_error = None
