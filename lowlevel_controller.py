#!/usr/bin/env python

import rospy


class LowLevelControl:
    def __init__(self):

        # vehicle params
        self.whl_base = 0.18

        # system params
        self.dt = 0.01 # something passed in from ROS?
        self.half_dt = 0.5 * self.dt
        self.min_dt = 0.0001
        self.dt_inv = 1 / self.dt

        self.scale_err_int_vel = 0
        self.scale_err_int_steer = 0

        # low pass filter params for omega
        self.al_omega = 0.75
        self.al_omega_sf1 = 1 - self.al_omega

        # velocity gains
        self.kp_vel = 1.0
        self.kd_vel = 1.0
        self.ki_vel = 1.0
        self.ki_vel_inv = 1 / self.ki_vel
        self.al_vel = 0.75 # alpha for a low pass filter
        self.al_vel_sf1 = 1 - self.al_vel # alpha vel subtracted from 1
        self.use_P_vel = 1
        self.use_D_vel = 1
        self.use_I_vel = 1
        self.sat_vel_min = -0.5
        self.sat_vel_max = 0.5
        self.err_vel_prev = 0.0 # previous time step error
        self.err_vel_int = 0.0 # integral of error

        # steering gains
        self.kp_steer = 1.0
        self.kd_steer = 1.0
        self.ki_steer = 1.0
        self.ki_steer_inv = 1 / self.ki_st
        self.al_steer = 0.1 # alpha for a low pass filter
        self.al_steer_sf1 = 1 - self.al_steer
        self.use_P_steer = 1
        self.use_D_steer = 1
        self.use_I_steer = 1
        self.sat_steer_min = -0.5
        self.sat_steer_max = 0.5
        self.err_steer_prev = 0.0
        self.err_steer_int = 0.0

        # subscribers
        # -- current velocity and steering angle
        self.vel_sub = rospy.Subscriber( "velocity", Velocity, self.getVel )
        self.omega_sub = rospy.Subscriber( "omega", Omega, self.Omega )
        # -- current desired velocity and steering angle
        self.vel_des_sub = rospy.Subscriber( "velocity", Velocity, self.getVelDes )
        self.steer_des_sub = rospy.Subscriber( "steer", Steer, self.getSteerDes )

        # publishers, vel_PWM, steer_PWM
        self.vel_pub = rospy.Publisher( "velocity", VelPWM, queue_size=1 )
        self.steer_pub = rospy.Publisher( "steer", SteerPWM, queue_size=1 )

        # run the pid loops
        self.vel_pid = rospy.Timer( rospy.Duration( 0.1), self.bPidRun )
        self.vel_pid = rospy.Timer( rospy.Duration( 0.1), self.dPidRun )

    #

    def getVel( self, msg ):
        self.vel_prev = self.vel_cur
        self.vel_cur = msg.velocity

        # add a low pass filter here, as we read the value, instead of in PID?
        self.vel_cur = self.al_vel * self.vel_cur + self.al_vel_sf1 * self.vel_prev
        #
    #
    def getOmega( self, msg ):
        # self.steer_prev = self.steer_cur
        # self.steer_cur = msg.steer

        self.omega_prev = self.omega_cur
        self.omega_cur = msg.omega

        # low pass filter the omega
        self.omega_cur = self.al_omega * self.omega_cur + self.al_omega_sf1 * self.omega_prev

        # add a low pass here, instead of in PID?
        self.steer_cur = self.steer_tp1 # steer time+1 (next steer angle, using current for other equations, as suggested by Dan )
        self.steer_tp1 = self.al_steer * self.steer_cur + self.al_steer_sf1 * atan( self.omega_cur * self.whl_base / self.vel_cur )
        #
    #
    def getVelDes( self, msg ):
        self.vel_des_cur = msg.velocity
        #
    #
    def getSteerDes( self, msg ):
        self.steer_des_cur = msg.steer
        #
    #

    def bPidRun( self, event ):
        self.err_vel_prev, self.err_vel_int, self.vel_PWM = self.bPid( self.vel_cur, self.vel_des_cur, self.kp_vel, self.kd_vel, self.ki_vel, self.ki_vel_inv, self.scale_err_int_vel, self.al_vel, self.use_P_vel, self.use_D_vel, self.use_I_vel, self.sat_vel_min, self.sat_vel_max, self.err_vel_prev, self.err_vel_int )
        self.err_steer_prev, self.err_steer_int, self.steer_PWM = bPid( self.steer_cur, self.steer_des_cur, self.kp_steer, self.kd_steer, self.ki_steer, self.ki_steer_inv, self.scale_err_int_steer, self.al_steer, self.use_P_steer, self.use_D_steer, self.use_I_steer, self.sat_steer_min, self.sat_steer_max, self.err_steer_prev, self.err_steer_int )
    #

    def dPidRun( self, event ):
        self.err_vel_prev, self.err_vel_int, self.vel_PWM = self.dPid( self.vel_cur, self.vel_des_cur, self.kp_vel, self.kd_vel, self.ki_vel, self.ki_vel_inv, self.use_P_vel, self.use_D_vel, self.use_I_vel, self.sat_vel_min, self.sat_vel_max, self.err_vel_prev, self.err_vel_int )
        self.err_steer_prev, self.err_steer_int, self.steer_PWM = dPid( self.steer_cur, self.steer_des_cur, self.kp_steer, self.kd_steer, self.ki_steer, self.ki_steer_inv, self.use_P_steer, self.use_D_steer, self.use_I_steer, self.sat_steer_min, self.sat_steer_max, self.err_steer_prev, self.err_steer_int )

    def bPid( self, cur, des_cur, kp, kd, ki, ki_inv, scale_err_int, al, use_P, use_D, use_I, sat_min, sat_max, err_prev, err_int ):

        # compute error
        err = des_cur - cur
        # simple derivative of error FIX THIS !!!!!
        err_der = self.dt_inv * ( err - err_prev )

        # P term
        if use_P:
            p_term = kp * err
        else:
            p_term = 0.0
        #
        # D term
        if use_D: # FIX THIS ... self.error????
            if not derivative is None:
                self.derivative = derivative
            elif dt > self.min_dt:
                self.derivative = (2.0*self.tau - dt)/(2.0*self.tau + dt)*self.derivative + 2.0/(2.0*self.tau + dt)*(error - self.last_error)
            else:
                self.derivative = 0.0
            #
            d_term = -kd * self.derivative
        else:
            d_term = 0.0
        #
        # compute PD control law, for assessment whether integral should be included
        pd_pre = p_term + d_term

        # I term
        if sat_min < pd_pre < sat_max:
            err_int = err_int + self.half_dt * ( err + err_prev)

            i_term = ki * err_int

            pdi_pre = pd_pre + i_term

            if not sat_min < pdi_pre:
                # i_term = sign( pd_pre ) * sat_min - pd_pre
                i_term = sat_min - pd_pre
                scale_err_int = 1
            elif not pdi_pre < sat_max:
                # i_term = sign( pd_pre ) * sat_max - pd_pre
                i_term = sat_max - pd_pre
                scale_err_int = 1
            #

            if scale_err_int:
                err_int = i_term * ki_inv
                scale_err_int = 0
            #
        else:
            err_int = 0.0
        #

        pid_out = p_term + d_term + i_term

        return err, err_int, pid_out
    #

    def dPID( self, cur, des_cur, kp, kd, ki, ki_inv, use_P, use_D, use_I, sat_min, sat_max, err_prev, err_int, derivative=None ):

        # compute error
        error = des_cur - cur

        # P term
        if use_P:
            p_term = kp * error
        else:
            p_term = 0.0
        #

        # D term
        if use_D: # FIX THIS !!!!!!
            if not derivative is None:
                self.derivative = derivative
            elif dt > 0.0001:
                self.derivative = (2.0*self.tau - dt)/(2.0*self.tau + dt)*self.derivative + 2.0/(2.0*self.tau + dt)*(error - self.last_error)
            else:
                self.derivative = 0.0
            d_term = -kd * self.derivative
        else:
            d_term = 0.0
        #

        # I term
        if use_I:
            err_int += error * self.dt
            i_term = ki * err_int
        else:
            i_term = 0.0
        #

        # combine
        u_com = p_term + d_term + i_term

        # saturate
        if u_com < sat_min:
            u_com = sat_min
        elif u_com > sat_max:
            u_com = sat_max
        # else:
        #

        # integrate anti-windup
        if use_I:
            if abs( p_term + d_term ) > abs( u_com ):
                err_int = 0.0
            else:
                err_int = ( u_com - p_term - d_term ) * ki_inv
            #
        #

        return err, err_int, u_com
    #

    # def velPID( self ):
    #     # compute error in velocity
    #     self.vel_err_cur = self.vel_des_cur - self.vel_cur
    #     #
    # #
    # def steerPID( self ):
    #     # compute error
    #     self.steer_err_cur = self.steer_des_cur - self.steer_cur
    #     # compute derivative of error
    #     self.steer_err_der_cur = ( self.steer_err_cur - self.steer_err_last ) / dt
    #
    #     # compute pd
    #     pd_pre = self.kp_st * self.steer_err_cur + self.kd_st * self.steer_err_der_cur
    #
    #     # saturate and anti-windup on integrator
    #     if abs( pd_out ) < 0.5:
    #         pi_pre =
    #         #
    #     #
    # #
#

if __name__ == 'main':
    rospy.init_node("lowlevel_controller", anonymous=False)
    control = LowLevelControl()
    rospy.spin()
    #
#





#
