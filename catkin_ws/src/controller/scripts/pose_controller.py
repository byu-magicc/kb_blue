from pid import PID
import numpy as np

class PoseController:
    def __init__(self):
        # stability is acheived for:
        # k_rho > 0, k_beta < 0, k_alpha - k_rho > 0
        self.rho_PID = PID(1.0, None, None, 0, 3)
        self.alpha_PID = PID(6.0, None, None, -np.pi/2, np.pi/2)
        self.beta_PID = PID(-2.0, None, None, -np.pi/2, np.pi/2)

        # direction
        self.D = 0

        # wheelbase [m]
        self.L = 0.18


    def run(self, goal, position, heading, dt):
        rho, alpha, beta = self.to_polar(goal, position, heading)

        # add desired heading
        beta += goal[2]

        velocity = self.rho_PID.run(rho, dt)
        omega = self.alpha_PID.run(alpha, dt) + self.beta_PID.run(beta, dt)

        # use the virtual unicycle to relate angular rate command omega to steering command
        steering = self.virtual_unicycle(omega, velocity)

        return velocity, steering


    def to_polar(self, goal, position, heading):
        pos_err = position - goal[:2]

        rho = np.linalg.norm(pos_err)

        if self.D == 0:
            beta = -np.arctan2(-pos_err[1], -pos_err[0])
            alpha = -(heading + beta)
            # negate the control law if the goal is behind the vehicle
            self.D = 1 if -np.pi/2 < alpha <= np.pi/2 else -1

        elif self.D == -1:
            beta = -np.arctan2(pos_err[1], pos_err[0])
            alpha = -(heading + beta)

        elif self.D == 1:
            beta = -np.arctan2(-pos_err[1], -pos_err[0])
            alpha = -(heading + beta)       

        # saturate alpha
        if alpha > np.pi/2:
            alpha = np.pi/2
        if alpha < -np.pi/2:
            alpha = -np.pi/2

        return rho, alpha, beta


    def virtual_unicycle(self, omega, velocity):
        steering = omega/np.abs(velocity)

        return steering