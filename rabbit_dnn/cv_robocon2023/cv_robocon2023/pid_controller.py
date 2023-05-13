import numpy as np


class PIDController:

    def __init__(self, kp, ki, kd, dt):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0


    def calculate_pid(self, error):

        proportional = self.kp * error[-1]
        integral = self.integral + self.ki * error[-1] * self.dt
        self.integral = integral
        derivative = self.kd * (error[-1]-error[-2])/self.dt

        output = proportional + integral + derivative

        return output
    