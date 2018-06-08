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

        # parameters
        self.nominal_delta = rospy.get_param("waypoint_follower/delta", 0.1)
        self.lead_distance = rospy.get_param("waypoint_follower/lead_distance", 1.0)
        self.follow_distance = rospy.get_param("waypoint_follower/follow_distance", 0.8)
        self.pose_close = rospy.get_param("pose_controller/close", 0.05)
        self.velocity_max = rospy.get_param("saturation/velocity_max", 1.0)
        self.velocity_min = rospy.get_param("saturation/velocity_min", -1.0)
        self.steering_max = rospy.get_param("saturation/steering_max", 0.35)

        # load mission
        self.waypoints_there = rospy.get_param("mission/waypoints_there")
        self.waypoints_back = rospy.get_param("mission/waypoints_back")
        self.landmark_home = rospy.get_param("mission/landmarks/home")
        self.landmark_brigham = rospy.get_param("mission/landmarks/brigham")
        self.final_heading = rospy.get_param("mission/endgame/heading")
        self.final_position_radius = rospy.get_param("mission/endgame/radius")

        # plotting stuff
        self.vehicle_marker = np.array([[0.1,-0.1,-0.03,-0.1,0.1],[0.0,0.07,0.0,-0.07,0.0]])
        self.plot_initialized = False

        self.circle_x = self.final_position_radius * np.cos(np.arange(0.0, 2*np.pi, 0.05))
        self.circle_y = self.final_position_radius * np.sin(np.arange(0.0, 2*np.pi, 0.05))

        # instantiate controllers
        self.waypoint_follower = WaypointFollower(self.nominal_delta, self.lead_distance, self.follow_distance)
        self.pose_controller = PoseController()

        # ROS pub/sub
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

        # saturate commands
        if self.velocity > self.velocity_max:
            self.velocity = self.velocity_max
        elif self.velocity < self.velocity_min:
            self.velocity = self.velocity_min

        if self.steering > self.steering_max:
            self.steering = self.steering_max
        elif self.steering < -self.steering_max:
            self.steering = -self.steering_max

        drive_msg = Drive()
        drive_msg.steering = self.steering
        drive_msg.velocity = self.velocity
        self.command_pub.publish(drive_msg)

        self.goal = self.waypoint_follower.get_goal()

    def control(self, position, heading, dt):
        velocity, steering = (0, 0)

        if self.state == Master.STATE_WAITING_FOR_POSE:
            self.waypoint_follower.set_waypoints(self.waypoints_there, position)

            self.state = Master.STATE_WAYPOINTS_THERE
            rospy.loginfo("Setting state to: WAYPOINTS_THERE")

        elif self.state == Master.STATE_WAYPOINTS_THERE:
            velocity, steering = self.waypoint_follower.update(position, heading, dt)

            if np.linalg.norm(position - self.landmark_brigham) < self.final_position_radius:
                self.state = Master.STATE_LINE_UP_THERE
                rospy.loginfo("Setting state to: LINE_UP_THERE")

        elif self.state == Master.STATE_LINE_UP_THERE:
            final_wp = self.waypoints_there[-1]
            goal = np.append(final_wp, [self.final_heading])

            velocity, steering = self.pose_controller.run(goal, position, heading, dt)

            if np.linalg.norm(position - final_wp) < self.pose_close:
                self.state = Master.STATE_KISS_BRIGHAM_THERE
                rospy.loginfo("Setting state to: KISS_BRIGHAM_THERE")

        elif self.state == Master.STATE_KISS_BRIGHAM_THERE:
            # TODO: Change condition
            if True:
                self.state = Master.STATE_KISS_BRIGHAM_BACK
                rospy.loginfo("Setting state to: KISS_BRIGHAM_BACK")

        elif self.state == Master.STATE_KISS_BRIGHAM_BACK:
            # TODO: Change condition
            if True:
                self.state = Master.STATE_LINE_UP_BACK
                rospy.loginfo("Setting state to: LINE_UP_BACK")

        elif self.state == Master.STATE_LINE_UP_BACK:

            # TODO: Change condition
            if True:
                self.waypoint_follower.set_waypoints(self.waypoints_back, position)

                self.state = Master.STATE_WAYPOINTS_BACK
                rospy.loginfo("Setting state to: WAYPOINTS_BACK")


        elif self.state == Master.STATE_WAYPOINTS_BACK:
            velocity, steering = self.waypoint_follower.update(position, heading, dt)

            # TODO: Better logic for "if I'm at the last waypoint"
            if np.linalg.norm(position - self.landmark_home) < self.lead_distance*1.1:
                self.state = Master.STATE_DONE
                rospy.loginfo("Setting state to: DONE")

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
        self.ax.axis([-2, 16, -2, 10])

        self.ax.plot([w[0] for w in self.waypoint_follower.waypoints], [w[1] for w in self.waypoint_follower.waypoints], 'bs', mec='none', alpha=0.3)

        if self.state == Master.STATE_WAYPOINTS_THERE or self.state == Master.STATE_WAYPOINTS_BACK:
            self.ax.plot(self.goal[0], self.goal[1], 'go')

        self.ax.plot(self.landmark_brigham[0], self.landmark_brigham[1], 'r*')
        self.ax.plot(self.circle_x + self.landmark_brigham[0], self.circle_y + self.landmark_brigham[1], 'r--')

        R = np.array([[np.cos(self.heading), np.sin(self.heading)],[-np.sin(self.heading), np.cos(self.heading)]]).T
        vehicle = R.dot(self.vehicle_marker) + np.atleast_2d(self.position).T
        self.ax.plot(vehicle[0,:], vehicle[1,:], 'k-')

        self.fig.canvas.draw()


if __name__ == '__main__':
    rospy.init_node("controller")
    m = Master()
    rospy.spin()
