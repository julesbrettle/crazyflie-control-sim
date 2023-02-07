
class Controller1D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (PIDGains dataclass):           pid gain class
        """
        self.params = cfparams

        # control gains
        self.kp_z = pid_gains.kp
        self.ki_z = pid_gains.ki
        self.kd_z = pid_gains.kd

        self.sum_error = 0

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust
        """

        # your code here
        e_pos = setpoint.z_pos - state.z_pos
        self.sum_error += e_pos
        e_vel = setpoint.z_vel - state.z_vel
        #self.sum_error += e_vel
        a_d = 0
        a_c = a_d + self.kd_z*e_vel + self.kp_z*e_pos + self.ki_z*self.sum_error + self.params.g
        U = self.params.mass*(a_c)

        return U
