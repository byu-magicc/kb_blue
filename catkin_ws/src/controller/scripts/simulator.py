#!/usr/bin/env python
import time

import rospy
from controller.msg import Drive
from geometry_msgs.msg import Pose2D
from kb_utils.msg import Encoder

import numpy as np

class Simulator:
    def __init__(self):
        self.L = 0.18 # wheelbase

        self.pos = np.zeros((2, 1))
        self.theta = np.pi/2

        self.v = 0.0 # velocity
        self.gamma = 0.0 # steering angle

        # What for plot to initialize before simulating car dynamics
        time.sleep(0.5)

        # encoder distance
        self.distance = 0
        self.last_pos = np.zeros((2,1))

        self.drive_sub = rospy.Subscriber("drive", Drive, self.drive_callback)
        self.pose_pub = rospy.Publisher("pose", Pose2D, queue_size=1)
        self.enc_pub = rospy.Publisher("encoder", Encoder, queue_size=1)
        self.dynamics_timer = rospy.Timer(rospy.Duration(0.01), self.dynamics_timer_callback)

    def init_plot(self):
        pass

    def drive_callback(self, msg):
        self.v = msg.velocity
        self.gamma = msg.steering

    def dynamics_timer_callback(self, event):
        if event.last_real:
            dt = (event.current_real - event.last_real).to_sec()
        else:
            dt = 0.0

        self.pos[0] += (self.v * np.cos(self.theta)) * dt
        self.pos[1] += (self.v * np.sin(self.theta)) * dt
        self.theta += (self.v / self.L * np.tan(self.gamma)) * dt

        # increment noisy encoder
        sigma = 0.05
        eta = sigma*np.random.randn()
        self.distance += np.sign(self.v)*np.linalg.norm(self.pos - self.last_pos) + eta
        self.last_pos = np.copy(self.pos)

        msg = Pose2D()
        msg.x = self.pos[0]
        msg.y = self.pos[1]
        msg.theta = self.theta
        self.pose_pub.publish(msg)

        msg = Encoder()
        msg.dist = self.distance
        self.enc_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("simulator", anonymous=False)
    sim = Simulator()
    rospy.spin()
