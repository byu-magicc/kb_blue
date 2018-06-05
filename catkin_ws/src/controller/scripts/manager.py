#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from tf.transformations import euler_from_quaternion

import numpy as np
import matplotlib.pyplot as plt

class Manager:
    def __init__(self):
        self.have_waypoints = False
        self.waypoints = np.zeros((2,0))
        self.path = np.zeros((2,0))
        self.goal = np.zeros((2,1))

        self.path_index = 0

        self.position = np.zeros((2,1))
        self.heading = 0.5

        self.nominal_delta = rospy.get_param("delta", 0.1)
        self.lead_distance = rospy.get_param("lead", 0.5)

        # TEMP
        # self.waypoints = np.array([[1.0,2.0,1.0],[0.0,1.0,2.0]])
        self.waypoints = [ np.array([[1.0],[0.0]]), np.array([[2.0],[1.0]]), np.array([[1.0],[2.0]]) ]
        self.generate_path()
        self.have_waypoints = True
        # end TEMP

        self.plot_initialized = False

        self.vehicle_marker = np.array([[0.1,-0.1,-0.03,-0.1,0.1],[0.0,0.07,0.0,-0.07,0.0]])

        self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        self.waypoints_sub = rospy.Subscriber("path", Path, self.waypoints_callback)

        self.goal_pub = rospy.Publisher("goal", PoseStamped, queue_size=1)

        self.plot_timer = rospy.Timer(rospy.Duration(0.1), self.plotting_callback)

    def pose_callback(self, msg):
        self.position = np.array([[msg.pose.position.x], [msg.pose.position.y]])
        _, _, self.heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        self.compute_goal()

        msg_out = PoseStamped()
        msg_out.header = msg.header
        msg_out.pose.position.x = self.goal[0]
        msg_out.pose.position.y = self.goal[1]
        self.goal_pub.publish(msg_out)

    def waypoints_callback(self, msg):
        self.waypoints = []
        for pose in msg.poses:
            self.waypoints.append(np.array([[pose.pose.position.x], [pose.pose.position.y]]))
        self.generate_path()

    def plotting_callback(self, event):
        if not self.plot_initialized:
            plt.ion()
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111)
            self.ax.set_title("Goal Manager")
            self.ax.axis('equal')

            # lines = self.ax.plot(self.waypoints[0,:],self.waypoints[1,:],'b*', [],[],'r-', [],[],'go', [],[],'k-')
            # lines = self.ax.plot([],[],'b*', [],[],'r-', [],[],'go', [],[],'k-')
            # self.line_waypoint = lines[0]
            # self.line_discretized = lines[1]
            # self.line_goal = lines[2]
            # self.line_vehicle = lines[3]

            self.plot_initialized = True

        if self.have_waypoints:
            self.ax.clear()

            # self.ax.plot(self.waypoints[0,:], self.waypoints[1,:], 'b*')
            self.ax.plot([w[0] for w in self.waypoints], [w[1] for w in self.waypoints], 'bs')
            self.ax.plot([p[0] for p in self.path], [p[1] for p in self.path], 'r.')

            self.ax.plot(self.goal[0], self.goal[1], 'go')

            R = np.array([[np.cos(self.heading), np.sin(self.heading)],[-np.sin(self.heading), np.cos(self.heading)]])
            vehicle = R.dot(self.vehicle_marker) + self.position
            self.ax.plot(vehicle[0,:], vehicle[1,:], 'k-')

            # self.line_waypoint.set_data(self.waypoints[0,:], self.waypoints[1,:])

            self.fig.canvas.draw()

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


if __name__ == '__main__':
    rospy.init_node("manager")
    m = Manager()
    rospy.spin()
