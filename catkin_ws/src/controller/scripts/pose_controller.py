from pid import PID
import numpy as np

class PoseController:
    def __init__(self):
        # stability is acheived for:
        # k_rho > 0, k_beta < 0, k_alpha - k_rho > 0
        self.rho_PID = PID(1.0, None, None, 0, 4)
        # self.rho_PID = PID(1.0, None, None, -35, 35)
        self.alpha_PID = PID(5.0, None, None, -np.pi/2, np.pi/2)
        self.beta_PID = PID(-2.0, None, None, -np.pi/2, np.pi/2)

        # direction
        self.D = 0

        # wheelbase [m]
        self.L = 0.18

    def run(self, goal, position, heading, dt):
        rho, alpha, beta = self.to_polar(goal, position, heading)

        beta += goal[2]

        velocity = self.rho_PID.run(rho, dt)
        omega = self.alpha_PID.run(alpha, dt) + self.beta_PID.run(beta, dt)

        steering = self.virtual_unicycle(omega, velocity)

        return velocity, steering

    def to_polar(self, goal, position, heading):
        pos_err = position - goal[:2]

        rho = np.linalg.norm(pos_err)

        if self.D == 0:
            alpha = -np.arctan2(pos_err[1], pos_err[0])
            beta = -heading - alpha

            if -np.pi/2 < alpha <= np.pi/2:
                print("going forwards")
                self.D = 1
            else:
                print("going backwards")
                self.D = -1

        elif self.D == -1:
            alpha = -np.arctan2(pos_err[1], pos_err[0])
            beta = -heading - alpha

        elif self.D == 1:
            alpha = -np.arctan2(-pos_err[1], -pos_err[0])
            beta = -heading - alpha

        if alpha > np.pi/2:
            alpha = np.pi/2
        if alpha < -np.pi/2:
            alpha = -np.pi/2

        return rho, alpha, beta

    def virtual_unicycle(self, omega, velocity):
        steering = omega/np.abs(velocity)

        return steering


# class PoseController:
#     def __init__(self):
#         # stability is acheived for:
#         # k_rho > 0, k_beta < 0, k_alpha - k_rho > 0
#         self.rho_PID = PID(1.0, None, None, 0, 3)
#         # self.rho_PID = PID(1.0, None, None, -35, 35)
#         self.alpha_PID = PID(5.0, None, None, -np.pi/2, np.pi/2)
#         self.beta_PID = PID(-2.0, None, None, -np.pi/2, np.pi/2)

#         # direction
#         self.D = 0

#         # wheelbase [m]
#         self.L = 0.18

#     def run(self, goal, position, heading, dt):
#         rho, alpha, beta, direction = self.to_polar(goal, position, heading)

#         velocity = self.rho_PID.run(rho, dt)
#         omega = self.alpha_PID.run(alpha, dt) + self.beta_PID.run(beta, dt)

#         steering = self.virtual_unicycle(omega, velocity)

#         # negate the control law if the goal is behind the vehicle
#         velocity *= direction
#         steering *= direction

#         return velocity, steering

#     def to_polar(self, goal, position, heading):
#         rho = np.linalg.norm(goal[:2] - position)
#         alpha = np.arctan2(goal[1] - position[1], goal[0] - position[0]) - heading
#         beta = -heading - alpha

#         # direction of motion: +1 or -1
#         # +1 assumes the goal frame is in front of the vehicle.
#         direction = 1 if -np.pi/2 < alpha <= np.pi/2 else -1
#         direction = 1

#         return rho, alpha, beta, direction

#     def virtual_unicycle(self, omega, velocity):
#         steering = omega/np.abs(velocity)

#         return steering