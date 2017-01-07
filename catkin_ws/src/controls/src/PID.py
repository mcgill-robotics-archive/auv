#!/usr/bin/env python


class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0,
                 Integrator_max=500, Integrator_min=-500):

        self.output = 0

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min

    def update(self, error):
        """
        Calculate PID output value for given error
        """
        if error is None:
            return self.output 

        self.P_value = self.Kp * error
        self.D_value = self.Kd * (error - self.Derivator)
        self.Derivator = error

        self.Integrator = self.Integrator + error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        self.output = self.P_value + self.I_value + self.D_value

        return self.output
