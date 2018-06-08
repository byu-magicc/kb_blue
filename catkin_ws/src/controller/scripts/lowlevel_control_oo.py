#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from controller.msg import Drive
from std_msgs.msg import Bool
from kb_utils.msg import Command, Encoder
from std_msgs.msg import Float64

from collections import deque

from math import atan
import numpy as np

# ======================================
# ======================================

class PID:
    def __init__( self, kp, kd, ki, sat_min, sat_max, tau=0.05 ):
        # gains
        self.kp = kp
        self.kd = kd
        self.ki = ki
        # control authority saturation limits
        self.sat_min = sat_min
        self.sat_max = sat_max
        # low pass filter strength and derivative bandwidth limiting factor
        self.tau = tau

        self.use_P = (self.kp != 0.0)
        self.use_D = (self.kd != 0.0)
        self.use_I = (self.ki != 0.0)

        # pre-compute ki_inverse for less computation at each function call
        if self.use_I:
            self.ki_inv = 1 / self.ki
        #
        # pre-compute for less computation in function
        self.tau2x = 2.0 * self.tau


        # 'static' of derivative for time=t-1
        self.derivative = 0.0
        # self.derivative_prev = 0.0
        self.err_integral = 0.0
        # self.integral_prev = 0.0

        # 'static' error of time=t-1
        # assign these in the subscriber
        self.val = 0.0
        self.val_prev = 0.0
        self.err = 0.0
        self.err_prev = 0.0
    #

    def compute_pid( self, dt, half_dt, val_cur, val_des_cur, override_active, derivative=None ):

        self.val_prev = self.val
        self.val = val_cur

        # compute error
        self.err_prev = self.err
        self.err = val_des_cur - val_cur

        # proportional term
        if self.use_P:
            p_term = self.kp * self.err
        else:
            p_term = 0.0

        # derivative term
        if self.use_D:
            if not derivative is None:
                self.derivative = derivative
            elif dt > 0.0001:
                self.derivative = ( self.tau2x - dt ) / ( self.tau2x + dt ) * self.derivative + 2.0 / ( self.tau2x + dt ) * ( self.val - self.val_prev )
            else:
                self.derivative = 0.0
            d_term = -self.kd * self.derivative
        else:
            d_term = 0.0
        #

        # ==============================

        # compute preliminary value from just pd control
        pd_pre = p_term + d_term


        # ==============================

        # integral term
        i_term = 0.0

        if self.sat_min < pd_pre < self.sat_max and not override_active:
            self.err_integral = self.err_integral + half_dt * ( self.err + self.err_prev )

            i_term = self.ki * self.err_integral

            pdi_pre = pd_pre + i_term

            if self.sat_min > pdi_pre:
                i_term = self.sat_min - pd_pre
                self.err_integral = i_term * self.ki_inv
            elif pdi_pre > self.sat_max:
                i_term = self.sat_max - pd_pre
                self.err_integral = i_term * self.ki_inv
            #
        else:
            self.err_integral = 0.0
        #

        pid_out = pd_pre + i_term

        return pid_out
    #
#

# ======================================
# ======================================

class LowLevelControl:
    def __init__( self ):

        # class member variables
        self.vel_cur = 0.0
        self.vel_prev = 0.0

        self.vel_des_filtered = 0.0
        self.vel_filter_tau = 0.2

        # self.omega_cur = 0.0
        # self.omega_prev = 0.0

        self.vel_des_cur = 0.0
        self.vel_des_prev = 0.0

        self.steer_cur = 0.0
        self.steer_prev = 0.0

        self.steer_des_cur = 0.0
        self.steer_des_prev = 0.0

        # raw values that are read from subscriber, to be filtered
        self.vel_raw = 0.0
        self.omega_raw = 0.0
        self.omega_raw_buffer = deque( maxlen=20 )

        # self.pid_timer_dt = 0.1
        self.override_active = True

        self.whl_base = 0.18

        # ======================================
        # create instance of pid class
        vel_kp = 0.45
        vel_kd = 0.0
        vel_ki = 0.6

        vel_sat_min = -0.5
        vel_sat_max = 0.5

        self.vel_alpha = 0.75
        self.vel_alpha_sf1 = 1.0 - self.vel_alpha
        vel_tau = 0.05

        self.vel_ctl = PID( vel_kp, vel_kd, vel_ki, vel_sat_min, vel_sat_max, vel_tau )

        steer_kp = 0.0
        steer_kd = 0.0
        steer_ki = 8.0

        steer_sat_min = -0.5
        steer_sat_max = 0.5

        self.steer_alpha = 0.01
        self.steer_alpha_sf1 = 1.0 - self.steer_alpha
        steer_tau = 0.05

        self.steer_ctl = PID( steer_kp, steer_kd, steer_ki, steer_sat_min, steer_sat_max, steer_tau )


        # ======================================

        self.time_prev = None

        # subscribers
        # -- current frame omega
        self.imu_sub = rospy.Subscriber( "imu/data", Imu, self.getOmega )
        # -- current velocity
        self.encoder_sub = rospy.Subscriber( "encoder", Encoder, self.getVel )
        # -- current desired velocity and steering
        self.drive_sub = rospy.Subscriber( "drive", Drive, self.getDesired )
        # -- safety pilot over ride boolean flag __'topic', mesage type, callback name__
        self.safe_override_sub = rospy.Subscriber( "safety_pilot_override", Bool, self.getSafe )

        # publisher, vel_com(mand), steer_com(mand)
        self.command_pub = rospy.Publisher( "command", Command, queue_size=1 )

        # debug publishers
        self.vel_filtered_pub = rospy.Publisher( "debug/vel_filtered", Float64, queue_size=1 )
        self.vel_des_filtered_pub = rospy.Publisher( "debug/vel_des_filtered", Float64, queue_size=1 )
        self.steering_angle_pub = rospy.Publisher( "debug/steering_angle", Float64, queue_size=1 )



    #

    def getVel( self, msg ):

        # compute dt and half_dt for compute_pid
        if not self.time_prev is None:
            t_cur = rospy.Time.now().to_sec()
            dt = ( t_cur - self.time_prev )
            self.time_prev = t_cur
        else:
            dt = 0.0
            self.time_prev = rospy.Time.now().to_sec()
        half_dt = 0.5 * dt

        # update current velocity measurement with low pass filter
        self.vel_prev = self.vel_cur
        self.vel_raw = msg.vel
        self.vel_cur = self.vel_alpha * self.vel_raw + self.vel_alpha_sf1 * self.vel_prev

        # input shaping for velocity command
        self.vel_des_filtered += 1.0/self.vel_filter_tau * (self.vel_des_cur - self.vel_des_filtered) * dt


        # inertialsense coordinate has z in opposite direction
        omega_cur_average = -np.mean( self.omega_raw_buffer )

        if abs(self.vel_cur) > 0.1:
            self.steer_cur = self.steer_alpha * self.steer_prev + self.steer_alpha_sf1 * atan( omega_cur_average * self.whl_base / self.vel_cur )
            self.steer_prev = self.steer_cur
        #

        # ======================================

        # compute control laws
        vel_cmd_out = self.vel_ctl.compute_pid( dt, half_dt, self.vel_cur, self.vel_des_filtered, self.override_active )

        steer_cmd_out = self.steer_ctl.compute_pid( dt, half_dt, self.steer_cur, self.steer_des_cur, self.override_active )

        cmd_msg = Command()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.throttle = vel_cmd_out
        cmd_msg.steer = steer_cmd_out
        self.command_pub.publish(cmd_msg)
        
        self.vel_filtered_pub.publish( self.vel_cur )
        self.vel_des_filtered_pub.publish( self.vel_des_filtered )
        self.steering_angle_pub.publish( self.steer_cur )

    #
    def getOmega( self, msg ):

        # self.omega_prev = self.omega_cur
        self.omega_raw = msg.angular_velocity.z

        self.omega_raw_buffer.append( self.omega_raw )

    #

    def getDesired( self, msg ):
        self.vel_des_cur = msg.velocity
        self.steer_des_cur = msg.steering
    #

    def getSafe( self, msg ):
        self.override_active = msg.data
    #

#


# ======================================
# ======================================

if __name__ == '__main__':
    rospy.init_node("lowlevel_control_oo")
    control = LowLevelControl()
    rospy.spin()

    #
#
