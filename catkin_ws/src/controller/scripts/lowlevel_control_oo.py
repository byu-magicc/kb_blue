#!/usr/bin/env python

import rospy


class PID:
    def __init__( self, kp, kd, ki, sat_min, sat_max, alpha, tau=0.05 ):
        # gains
        self.kp = kp
        self.kd = kd
        self.ki = ki
        # control authority saturation limits
        self.sat_min = sat_min
        self.sat_max = sat_max
        # low pass filter strength and derivative bandwidth limiting factor
        self.alpha = alpha
        self.tau = tau

        self.use_P = (self.kp != 0.0)
        self.use_D = (self.kd != 0.0)
        self.use_I = (self.ki != 0.0)

        # pre-compute ki_inverse for less computation at each function call
        if self.use_I:
            self.ki_inv = 1 / self.ki
        #
        # pre-compute ( 1 - alpha ) for less computation in function
        # alpha subtracted from 1
        self.alpha_sf1 = 1.0 - self.alpha
        self.tau2x = 2.0 * self.tau
        # create dt and half_dt, will BOTH be assigned in SUBSCRIBER callback
        self.dt = 0.01
        self.half_dt = 0.5 * self.dt
        # derivative first term scaling value
        self.der_term1_scale = ( self.tau2x - ctl.dt ) / ( self.tau2x + ctl.dt )
        self.der_term2_scale = 2.0 / ( self.tau2x + ctl.dt )

        # 'static' of derivative for time=t-1
        self.err_derivative = 0.0
        self.err_derivative_prev = 0.0
        self.err_integral = 0.0
        # self.integral_prev = 0.0

        # 'static' error of time=t-1
        # assign these in the subscriber
        self.err = 0.0
        self.err_prev = 0.0
    #

    def compute_pid( self, derivative=None ):

        # proportional term
        if self.use_P:
            p_term = self.kp * self.err
        else:
            p_term = 0.0

        # derivative term
        if self.use_D:
            if not derivative is None:
                self.derivative = derivative
            elif ctl.dt > 0.0001:
                self.err_derivative = self.der_term1_scale * self.err_derivative + self.der_term2_scale * ( self.err - self.err_prev )
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
        if self.sat_min < pd_pre < self.sat_max:
            self.err_integral = self.err_integral + self.half_dt * ( self.err + self.err_prev )

            i_term = self.ki * self.err_integral

            pdi_pre = pd_pre + i_term

            if not sat_min < sat_max:

        #
    #
#
























#
