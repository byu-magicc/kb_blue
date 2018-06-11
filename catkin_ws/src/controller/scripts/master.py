#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from controller.msg import Drive
from kb_utils.msg import Encoder

import numpy as np
import matplotlib.pyplot as plt

from waypoint_follower import WaypointFollower
from pose_controller import PoseController
from kiss_controller import KissController


class Master:
    STATE_WAITING_FOR_POSE = 0
    STATE_WAYPOINTS_THERE = 1
    STATE_LINE_UP_THERE = 2
    STATE_RETRY_LINE_UP_THERE = 3
    STATE_KISS_BRIGHAM_THERE = 4
    STATE_KISS_BRIGHAM_BACK = 5
    STATE_LINE_UP_BACK = 6
    STATE_WAYPOINTS_BACK = 7
    STATE_DONE = 8

    def __init__(self):
        self.state = self.STATE_WAITING_FOR_POSE

        self.position = np.zeros((2,))
        self.heading = 0.0
        self.goal = np.zeros((2,))

        self.velocity = 0.0
        self.steering = 0.0

        self.encoder_distance = 0.0
        self.encoder_start_kiss = 0.0
        self.encoder_start_retry = 0.0
        self.encoder_start_line_up_back = 0.0

        self.last_update_time = None

        # parameters
        self.nominal_delta = rospy.get_param("waypoint_follower/delta", 0.1)
        self.lead_distance = rospy.get_param("waypoint_follower/lead_distance", 1.0)
        self.follow_distance = rospy.get_param("waypoint_follower/follow_distance", 0.8)
        self.pucker_close_distance = rospy.get_param("pose_controller/close_position", 0.05)
        self.pucker_close_heading = rospy.get_param("pose_controller/close_heading", 0.17)
        self.retry_distance = rospy.get_param("retry_controller/distance", 2.0)
        self.retry_close = rospy.get_param("retry_controller/close", 0.10)
        self.kiss_close = rospy.get_param("kiss_controller/close", 0.10)
        self.line_up_back_distance = rospy.get_param("line_up_back_controller/distance", 3.0)
        self.line_up_back_close = rospy.get_param("line_up_back_controller/close", 0.10)
        self.velocity_max = rospy.get_param("saturation/velocity_max", 1.0)
        self.velocity_min = rospy.get_param("saturation/velocity_min", -1.0)
        self.steering_max = rospy.get_param("saturation/steering_max", 0.2)
        self.plot_gui = rospy.get_param("gui/plot", True)

        # load mission
        self.waypoints_there = rospy.get_param("mission/waypoints_there")
        self.waypoints_back = rospy.get_param("mission/waypoints_back")
        self.landmark_home = rospy.get_param("mission/landmarks/home")
        self.landmark_brigham = rospy.get_param("mission/landmarks/brigham")
        self.pucker_pose = rospy.get_param("mission/pucker_pose")
        self.final_position_radius = rospy.get_param("mission/endgame/radius")

        # calculate distance between brigham and pucker pose
        self.pucker_dist = np.linalg.norm(np.array(self.landmark_brigham) - np.array(self.pucker_pose[:2]))

        # plotting stuff
        self.vehicle_marker = np.array([[0.1,-0.1,-0.03,-0.1,0.1],[0.0,0.07,0.0,-0.07,0.0]])
        self.plot_initialized = False

        self.circle_x = self.final_position_radius * np.cos(np.arange(0.0, 2*np.pi, 0.05))
        self.circle_y = self.final_position_radius * np.sin(np.arange(0.0, 2*np.pi, 0.05))

        # instantiate controllers
        self.waypoint_follower = WaypointFollower(self.nominal_delta, self.lead_distance, self.follow_distance)
        self.pose_controller = PoseController()
        self.kiss_controller = KissController()

        # ROS pub/sub
        self.command_pub = rospy.Publisher("drive", Drive, queue_size=1)
        self.pose_sub = rospy.Subscriber("pose", Pose2D, self.pose_callback)
        self.enc_sub = rospy.Subscriber("encoder", Encoder, self.encoder_callback)

        if self.plot_gui:
            self.plot_timer = rospy.Timer(rospy.Duration(0.1), self.plotting_callback)

    def encoder_callback(self, msg):
        self.encoder_distance = msg.dist

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
            waypoint_list = self.waypoints_there + [self.pucker_pose[:2]]
            self.waypoint_follower.set_waypoints(waypoint_list, position)

            self.state = Master.STATE_WAYPOINTS_THERE
            rospy.loginfo("Setting state to: WAYPOINTS_THERE")

        elif self.state == Master.STATE_WAYPOINTS_THERE:
            velocity, steering = self.waypoint_follower.update(position, heading, dt)

            if np.linalg.norm(position - self.pucker_pose[:2]) < self.final_position_radius:
                self.state = Master.STATE_LINE_UP_THERE
                rospy.loginfo("Setting state to: LINE_UP_THERE")

        elif self.state == Master.STATE_LINE_UP_THERE:
            velocity, steering = self.pose_controller.run(self.pucker_pose, position, heading, dt)

            if np.linalg.norm(position - self.pucker_pose[:2]) < self.pucker_close_distance:
                if abs(heading - self.pucker_pose[2]) < self.pucker_close_heading:
                    self.encoder_start_kiss = self.encoder_distance
                    self.state = Master.STATE_KISS_BRIGHAM_THERE
                    rospy.loginfo("Setting state to: KISS_BRIGHAM_THERE")
                else:
                    self.encoder_start_retry = self.encoder_distance
                    self.state = Master.STATE_RETRY_LINE_UP_THERE
                    rospy.loginfo("Setting state to: RETRY_LINE_UP_THERE")

        elif self.state == Master.STATE_RETRY_LINE_UP_THERE:
            relative_enc = self.encoder_distance - self.encoder_start_retry

            velocity, steering = self.kiss_controller.run(-self.retry_distance, relative_enc, dt)

            if np.abs(relative_enc + self.retry_distance) < self.retry_close:
                self.state = Master.STATE_LINE_UP_THERE
                rospy.loginfo("Setting state to: LINE_UP_THERE")

        elif self.state == Master.STATE_KISS_BRIGHAM_THERE:
            relative_enc = self.encoder_distance - self.encoder_start_kiss

            velocity, steering = self.kiss_controller.run(self.pucker_dist, relative_enc, dt)

            if np.abs(self.pucker_dist - relative_enc) < self.kiss_close:
                self.state = Master.STATE_KISS_BRIGHAM_BACK
                rospy.loginfo("Setting state to: KISS_BRIGHAM_BACK")

        elif self.state == Master.STATE_KISS_BRIGHAM_BACK:
            relative_enc = self.encoder_distance - self.encoder_start_kiss

            velocity, steering = self.kiss_controller.run(0, relative_enc, dt)

            if np.abs(relative_enc) < self.kiss_close:
                self.encoder_start_line_up_back = self.encoder_distance
                self.state = Master.STATE_LINE_UP_BACK
                rospy.loginfo("Setting state to: LINE_UP_BACK")

        elif self.state == Master.STATE_LINE_UP_BACK:
            relative_enc = self.encoder_distance - self.encoder_start_line_up_back

            velocity, steering = self.kiss_controller.run(-self.line_up_back_distance, relative_enc, dt)

            if np.abs(relative_enc + self.line_up_back_distance) < self.line_up_back_close:
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
        self.ax.axis([-2, 20, -15, 5])

        self.ax.plot([w[0] for w in self.waypoint_follower.waypoints], [w[1] for w in self.waypoint_follower.waypoints], 'bs', mec='none', alpha=0.3)

        if self.state == Master.STATE_WAYPOINTS_THERE or self.state == Master.STATE_WAYPOINTS_BACK:
            self.ax.plot(self.goal[0], self.goal[1], 'go')

        self.ax.plot(self.landmark_brigham[0], self.landmark_brigham[1], 'r*')
        self.ax.plot(self.circle_x + self.pucker_pose[0], self.circle_y + self.pucker_pose[1], 'r--')

        R = np.array([[np.cos(self.heading), np.sin(self.heading)],[-np.sin(self.heading), np.cos(self.heading)]]).T
        vehicle = R.dot(self.vehicle_marker) + np.atleast_2d(self.position).T
        self.ax.plot(vehicle[0,:], vehicle[1,:], 'k-')

        self.fig.canvas.draw()


if __name__ == '__main__':
    rospy.init_node("controller")
    m = Master()
    rospy.spin()
