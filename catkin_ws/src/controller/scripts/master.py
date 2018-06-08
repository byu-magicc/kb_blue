#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from controller.msg import Drive

import numpy as np
import matplotlib.pyplot as plt

from waypoint_follower import WaypointFollower
from pose_controller import PoseController


class Master:
    STATE_WAITING_FOR_POSE = 0
    STATE_WAYPOINTS_THERE = 1
    STATE_LINE_UP_THERE = 2
    STATE_KISS_BRIGHAM_THERE = 3
    STATE_KISS_BRIGHAM_BACK = 4
    STATE_LINE_UP_BACK = 5
    STATE_WAYPOINTS_BACK = 6
    STATE_DONE = 7

    def __init__(self):
        self.state = self.STATE_WAITING_FOR_POSE

        self.position = np.zeros((2,))
        self.heading = 0.0
        self.goal = np.zeros((2,))

        self.velocity = 0.0
        self.steering = 0.0

        self.last_update_time = None

        # controllers
        self.waypoint_follower = WaypointFollower()
        self.pose_controller = PoseController()

        # waypoints
        self.waypoints_there = [ np.array([5.0,0.0]), np.array([0.0,5.0]), np.array([5.0,9.5]) ]

        # endgame
        self.waypoint_brigham = np.array([5.0,10.0])
        self.final_heading = np.pi/2
        self.final_position_radius = 1.5

        # plotting stuff
        self.vehicle_marker = np.array([[0.1,-0.1,-0.03,-0.1,0.1],[0.0,0.07,0.0,-0.07,0.0]])
        self.plot_initialized = False

        self.circle_x = self.final_position_radius * np.cos(np.arange(0.0, 2*np.pi, 0.05))
        self.circle_y = self.final_position_radius * np.sin(np.arange(0.0, 2*np.pi, 0.05))

        # ROS
        self.pose_sub = rospy.Subscriber("pose", Pose2D, self.pose_callback)
        self.command_pub = rospy.Publisher("drive", Drive, queue_size=1)
        self.plot_timer = rospy.Timer(rospy.Duration(0.1), self.plotting_callback)

    def pose_callback(self, msg):
        if self.last_update_time:
            dt = (rospy.Time.now() - self.last_update_time).to_sec()
        else:
            dt = 0.0
        self.last_update_time = rospy.Time.now()

        self.position = np.array([msg.x, msg.y])
        self.heading = msg.theta

        # run the control state machine
        self.velocity, self.steering = self.control(self.position, self.heading, dt)

        drive_msg = Drive()
        drive_msg.steering = self.steering
        drive_msg.velocity = self.velocity
        self.command_pub.publish(drive_msg)

        self.goal = self.waypoint_follower.get_goal()

    def control(self, position, heading, dt):
        velocity, steering = (0, 0)

        if self.state == Master.STATE_WAITING_FOR_POSE:
            self.waypoint_follower.set_waypoints(self.waypoints_there)

            self.state = Master.STATE_WAYPOINTS_THERE
            rospy.loginfo("Setting state to: WAYPOINTS_THERE")

        elif self.state == Master.STATE_WAYPOINTS_THERE:
            velocity, steering = self.waypoint_follower.update(position, heading, dt)

            if np.linalg.norm(position - self.waypoint_brigham) < self.final_position_radius:
                self.state = Master.STATE_LINE_UP_THERE
                rospy.loginfo("Setting state to: LINE_UP_THERE")

        elif self.state == Master.STATE_LINE_UP_THERE:
            final_wp = self.waypoints_there[-1]
            goal = np.append(final_wp, [self.final_heading])

            velocity, steering = self.pose_controller.run(goal, position, heading, dt)

            if np.linalg.norm(position - final_wp) < 0.05:
                self.state = Master.STATE_KISS_BRIGHAM_THERE
                rospy.loginfo("Setting state to: KISS_BRIGHAM_THERE")

        elif self.state == Master.STATE_KISS_BRIGHAM_THERE:
            pass

        elif self.state == Master.STATE_KISS_BRIGHAM_BACK:
            pass

        elif self.state == Master.STATE_LINE_UP_BACK:
            pass

        elif self.state == Master.STATE_WAYPOINTS_BACK:
            pass

        elif self.state == Master.STATE_DONE:
            pass

        return velocity, steering

    def plotting_callback(self, event):
        if not self.plot_initialized:
            plt.ion()
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111)
            self.ax.set_title("Controller")
            self.ax.axis('equal')

            self.plot_initialized = True

        self.ax.clear()
        self.ax.axis([-2, 12, -2, 12])

        self.ax.plot([w[0] for w in self.waypoint_follower.waypoints], [w[1] for w in self.waypoint_follower.waypoints], 'bs', mec='none', alpha=0.3)

        if self.state == Master.STATE_WAYPOINTS_THERE or self.state == Master.STATE_WAYPOINTS_BACK:
            self.ax.plot(self.goal[0], self.goal[1], 'go')

        self.ax.plot(self.waypoint_brigham[0], self.waypoint_brigham[1], 'r*')
        self.ax.plot(self.circle_x + self.waypoint_brigham[0], self.circle_y + self.waypoint_brigham[1], 'r--')

        R = np.array([[np.cos(self.heading), np.sin(self.heading)],[-np.sin(self.heading), np.cos(self.heading)]]).T
        vehicle = R.dot(self.vehicle_marker) + np.atleast_2d(self.position).T
        self.ax.plot(vehicle[0,:], vehicle[1,:], 'k-')

        self.fig.canvas.draw()


if __name__ == '__main__':
    rospy.init_node("controller")
    m = Master()
    rospy.spin()
