#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from controller.msg import Drive
from nav_msgs.msg import Path

# from tf.transformations import euler_from_quaternion

import numpy as np
import matplotlib.pyplot as plt


class PID:
    def __init__(self, kp, kd, ki, min, max, tau=0.05):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.min = min
        self.max = max
        self.tau = tau

        self.use_P = (self.kp != 0.0)
        self.use_D = (self.kd != 0.0)
        self.use_I = (self.ki != 0.0)

        self.derivative = 0.0
        self.integral = 0.0

        self.last_error = 0.0

    def run(self, error, dt, derivative=None):

        # P term
        if self.use_P:
            p_term = self.kp * error
        else:
            p_term = 0.0

        # D term
        if self.use_D:
            if not derivative is None:
                self.derivative = derivative
            elif dt > 0.0001:
                self.derivative = (2.0*self.tau - dt)/(2.0*self.tau + dt)*self.derivative + 2.0/(2.0*self.tau + dt)*(error - self.last_error)
            else:
                self.derivative = 0.0
            d_term = -self.kd * self.derivative
        else:
            d_term = 0.0

        # I term
        if self.use_I:
            self.integral += error * dt
            i_term = self.ki * self.integral
        else:
            i_term = 0.0

        # combine
        u = p_term + d_term + i_term

        # saturate
        if u < self.min:
            u_sat = self.min
        elif u > self.max:
            u_sat = self.max
        else:
            u_sat = u

        # integrator anti-windup
        # if u != u_sat and self.use_I and abs(i_term) > abs(u - p_term - d_term):
        #     self.integral = (u_sat - p_term - d_term)/self.ki
        if self.use_I:
            if abs(p_term + d_term) > abs(u_sat): # PD is already saturating, so set integrator to 0 but don't let it run backwards
                self.integral = 0
            else: # otherwise only let integral term at most take us just up to saturation
                self.integral = (u_sat - p_term - d_term) / self.ki

        # bookkeeping
        self.last_error = error

        return u_sat


class TrajectoryController:
    def __init__(self, follow_distance):
        self.d = follow_distance
        self.angle_PID = PID(1.0, 0, 0, -0.35, 0.35)
        self.distance_PID = PID(5.0, 0, 0.0, 0.0, 3.0)

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
        self.lead_distance = rospy.get_param("lead_distance", 1.0)
        self.follow_distance = rospy.get_param("follow_distance", 0.8)

        self.trajectory_controller = TrajectoryController(self.follow_distance)
        self.last_controller_time = None

        # TEMP
        #self.waypoints = [ np.array([[1.0],[0.0]]), np.array([[2.0],[1.0]]), np.array([[1.0],[2.0]]) ]
        # self.waypoints = [ np.array([[10.0],[0.0]]), np.array([[20.0],[10.0]]), np.array([[10.0],[20.0]]) ]
        self.waypoints = [ np.array([[-1.0],[-0.6]]), np.array([[0.5],[0.5]]), np.array([[2.25],[0.5]]) ]
        self.generate_path()
        self.have_waypoints = True
        # end TEMP

        self.plot_initialized = False

        self.vehicle_marker = np.array([[0.1,-0.1,-0.03,-0.1,0.1],[0.0,0.07,0.0,-0.07,0.0]])

        self.pose_sub = rospy.Subscriber("pose", Pose2D, self.pose_callback)
        self.waypoints_sub = rospy.Subscriber("path", Path, self.waypoints_callback)

        self.goal_pub = rospy.Publisher("goal", Pose2D, queue_size=1)
        self.command_pub = rospy.Publisher("drive", Drive, queue_size=1)

        self.plot_timer = rospy.Timer(rospy.Duration(0.1), self.plotting_callback)

        self.initialized = False

    def pose_callback(self, msg):
        if self.last_controller_time:
            dt = (rospy.Time.now() - self.last_controller_time).to_sec()
        else:
            dt = 0.0
        self.last_controller_time = rospy.Time.now()

        # self.position = np.array([[msg.pose.position.x], [msg.pose.position.y]])
        # _, _, self.heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.position = np.array([[msg.x], [msg.y]])
        self.heading = msg.theta

        if not self.initialized:
            self.generate_path()
            self.initialized = True

        self.compute_goal()
        self.run_controller(dt)

        drive_msg = Drive()
        drive_msg.steering = self.steering
        drive_msg.velocity = self.velocity
        self.command_pub.publish(drive_msg)

        goal_msg = Pose2D()
        goal_msg.x = self.goal[0]
        goal_msg.y = self.goal[1]
        self.goal_pub.publish(goal_msg)

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

            self.ax.plot([w[0] for w in self.waypoints], [w[1] for w in self.waypoints], 'bs')
            self.ax.plot([p[0] for p in self.path], [p[1] for p in self.path], 'r.')

            self.ax.plot(self.goal[0], self.goal[1], 'go')

            R = np.array([[np.cos(self.heading), np.sin(self.heading)],[-np.sin(self.heading), np.cos(self.heading)]]).T
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

    def run_controller(self, dt):
        self.velocity, self.steering = self.trajectory_controller.run(self.goal, self.position, self.heading, dt)


if __name__ == '__main__':
    rospy.init_node("manager")
    m = Manager()
    rospy.spin()
