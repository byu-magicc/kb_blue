#!/usr/bin/env python

import rospy
from kb_utils.msg import Command

import numpy as np

class Simulator:
    def __init__(self):
        self.L = 0.18 # wheelbase

        self.pos = np.zeros((2, 1))
        self.theta = 0.0

        self.v = 0.1 # velocity
        self.gamma = 0.01 # steering angle

        self.command_sub = rospy.Subscriber("command", Command, self.command_callback)
        self.dynamics_timer = rospy.Timer(rospy.Duration(0.01), self.dynamics_timer_callback)

    def init_plot(self):
        pass

    def command_callback(self, msg):
        self.v = msg.throttle
        self.gamma = msg.steer

    def dynamics_timer_callback(self, event):
        if event.last_real:
            dt = (event.current_real - event.last_real).to_sec()
        else:
            dt = 0.0

        self.pos[0] += (self.v * np.cos(self.theta)) * dt
        self.pos[1] += (self.v * np.sin(self.theta)) * dt
        self.theta += (self.v / self.L * np.tan(self.theta)) * dt

if __name__ == '__main__':
    rospy.init_node("simulator", anonymous=False)
    sim = Simulator()
    rospy.spin()
