from pid import PID
import numpy as np

class TrajectoryController:
    def __init__(self, follow_distance):
        self.d = follow_distance
        self.angle_PID = PID(0.2, None, None, -0.35, 0.35)
        self.distance_PID = PID(5.0, None, 0.0, 0.0, 3.0)

    def run(self, goal, position, heading, dt):
        distance_error = np.linalg.norm(goal - position) - self.d

        desired_heading = np.arctan2(goal[1] - position[1], goal[0] - position[0])
        angle_error = desired_heading - heading

        while angle_error > np.pi:
            angle_error -= 2*np.pi
        while angle_error < -np.pi:
            angle_error += 2*np.pi

        steering = self.angle_PID.run(angle_error, dt)
        velocity = self.distance_PID.run(distance_error, dt)

        return velocity, steering
