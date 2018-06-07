import rospy
from trajectory_controller import TrajectoryController

import numpy as np
import matplotlib.pyplot as plt

class WaypointFollower:
    def __init__(self):
        self.have_waypoints = False
        self.waypoints = []
        self.path = []
        self.goal = np.zeros((2,))

        self.path_index = 0

        self.position = np.zeros((2,))
        self.heading = 0.0

        self.nominal_delta = rospy.get_param("delta", 0.1)
        self.lead_distance = rospy.get_param("lead_distance", 1.0)
        self.follow_distance = rospy.get_param("follow_distance", 0.8)

        self.trajectory_controller = TrajectoryController(self.follow_distance)

    def set_waypoints(self, waypoints, plot=False):
        self.waypoints = waypoints
        self.generate_path()
        self.have_waypoints = True
        if plot:
            self.plot()

    def update(self, position, heading, dt):
        self.position = position
        self.heading = heading

        self.compute_goal()
        self.velocity, self.steering = self.trajectory_controller.run(self.goal, self.position, self.heading, dt)

        return self.velocity, self.steering

    def get_goal(self):
        return self.path[self.path_index]

    def generate_path(self):
        self.path = [self.position]
        prev_wp = np.copy(self.position)

        for wp in self.waypoints:
            diff = wp - prev_wp
            num_steps = int(np.ceil(np.linalg.norm(diff) / self.nominal_delta))
            delta = 1.0/num_steps * diff

            for i in xrange(num_steps):
                self.path.append(prev_wp + (i+1)*delta)

            prev_wp = np.copy(wp)

        self.path_index = 0
        self.compute_goal()

    def compute_goal(self):
        while np.linalg.norm(self.path[self.path_index] - self.position) < self.lead_distance and self.path_index < len(self.path)-1:
            self.path_index += 1
        self.goal = self.path[self.path_index]

    def plot(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Waypoint Follower")
        self.ax.axis('equal')

        if self.have_waypoints:
            self.ax.clear()

            self.ax.plot([w[0] for w in self.waypoints], [w[1] for w in self.waypoints], 'bs')
            self.ax.plot([p[0] for p in self.path], [p[1] for p in self.path], 'r.')

            self.ax.plot(self.goal[0], self.goal[1], 'go')

            # R = np.array([[np.cos(self.heading), np.sin(self.heading)],[-np.sin(self.heading), np.cos(self.heading)]]).T
            # vehicle = R.dot(self.vehicle_marker) + self.position
            # self.ax.plot(vehicle[0,:], vehicle[1,:], 'k-')

            # self.line_waypoint.set_data(self.waypoints[0,:], self.waypoints[1,:])

            self.fig.canvas.draw()
