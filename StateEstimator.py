import numpy as np

from utils import State

class StateEstimator1D():
    """
    This class estimates the state (position and velocity) of the system based 
    on noisy sensor measurements using a kalman filter

    You are to implement the "compute" method.
    """
    def __init__(self, params, init_state):
        """
        Inputs:
        - params (CrazyflieParams dataclass):       model parameter class for the crazyflie
        - init_state (State dataclass):             initial state of the system
        """

        # your code here

        self.params = params
        self.init_state = init_state
        self.prev_state = init_state

        self.std = 0
        self.std_f = .1
        self.std_z = .1
        

    def compute(self, z_meas, U, time_delta):
        """
        Estimates the current system state given the noisy measurement and control input

        Inputs:
        - z_meas (float):       current noisy measurement from the system sensor
        - U (float):            computed control input (thrust)
        - time_delta (float):   discrete time interval for simulation

        Returns:
        - filtered_state (State): estimated system state (z_pos, z_vel) using the kalman filter

        """
        filtered_state = State()

        a = U / self.params.mass - self.params.g # system acceleration

        # PREDICTION

        mu_x = self.prev_state.z_pos + self.prev_state.z_vel * time_delta + .5 * a * time_delta ** 2
        std_x_sq = self.std ** 2 + self.std_f ** 2

        # UPDATE

        # calculate K (kalman gain)

        K = std_x_sq / (std_x_sq + self.std_z ** 2)

        # calculate y (residual)

        y = z_meas - mu_x

        # correct prediction with measurement

        filtered_state.z_pos = mu_x + K * y
        filtered_state.z_vel = self.prev_state.z_vel + a * time_delta

        self.prev_state = filtered_state

        self.std = (std_x_sq * self.std_z) / (std_x_sq + self.std_z)

        return filtered_state
