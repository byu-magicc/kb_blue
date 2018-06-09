from pid import PID
import numpy as np

class KissController:
    def __init__(self):
        self.distance_PID = PID(1.0, None, None, -0.5, 0.5)

    def run(self, distance_desired, distance, dt):

        error = distance_desired - distance

        velocity = self.distance_PID.run(error, dt)

        steering = 0

        return velocity, steering
