#!/usr/bin/env python

import rospy



# ======================================
# ======================================

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
        i_term = 0.0

        if self.sat_min < pd_pre < self.sat_max:
            self.err_integral = self.err_integral + self.half_dt * ( self.err + self.err_prev )

            i_term = self.ki * self.err_integral

            pdi_pre = pd_pre + i_term

            if not sat_min < pdi_pre:
                i_term = sat_min - pd_pre
                scale_err_int_flag = 1
            elif not pdi_pre < sat_max:
                i_term = sat_max - pd_pre
                scale_err_int_flag = 1
            #

            if scale_err_int_flag:
                self.err_integral = i_term * ki_inv
                scale_err_int_flag = 0
            #

        else:
            self.err_integral = 0.0
        #

        pid_out = pd_pre + i_term

        # self.publish( pid_out )
        return pid_out

        #
    #
#

# ======================================
# ======================================

class LowLevelControl:
    def __init__( self ):

        # class member variables
        self.vel_cur = 0.0
        self.vel_prev = 0.0

        self.omega_cur = 0.0
        self.omega_prev = 0.0

        self.vel_des_cur = 0.0
        self.vel_des_prev = 0.0

        self.steer_des_cur = 0.0
        self.steer_des_prev = 0.0

        self.pid_timer_dt = 0.1

        # ======================================
        # create instance of pid class
        vel_kp = 1.0
        vel_kd = 1.0
        vel_ki = 1.0

        vel_sat_min = -0.5
        vel_sat_max = 0.5

        vel_alpha = 0.75
        vel_tau = 0.05

        self.vel_ctl = PID( vel_kp, vel_kd, vel_ki, vel_sat_min, vel_sat_max, vel_alpha, vel_tau )

        st_kp = 1.0
        st_kd = 1.0
        st_ki = 1.0

        st_sat_min = -0.5
        st_sat_max = 0.5

        st_alpha = 0.01
        st_tau = 0.05

        self.steer_ctl = PID( st_kp, st_kd, st_ki, st_sat_min, st_sat_max, st_alpha, st_tau )


        # ======================================

        # subscribers
        # -- current velocity and steering angle
        self.vel_sub = rospy.Subscriber( "velocity", Velocity, self.getVel )
        self.omega_sub = rospy.Subscriber( "omega", Omega, self.getOmega )
        # -- current desired velocity and steering angle
        self.vel_des_sub = rospy.Subscriber( "velocity", Velocity, self.getVelDes )
        self.steer_des_sub = rospy.Subscriber( "steer", Steer, self.getSteerDes )

        # publishers, vel_com(mand), steer_com(mand)
        self.vel_pub = rospy.Publisher( "velocity", VelPWM, queue_size=1 )
        self.steer_pub = rospy.Publisher( "steer", SteerPWM, queue_size=1 )

        # # run the pid loops
        # self.vel_pid = rospy.Timer( rospy.Duration( self.pid_timer_dt ), self.bPidRun )
        # self.vel_pid = rospy.Timer( rospy.Duration( self.pid_timer_dt ), self.dPidRun )

    #

    def getVel( self, msg ):
        self.vel_prev = self.vel_cur
        self.vel_cur = msg.velocity

        # add a low pass filter here, as we read the value, instead of in PID?
        self.vel_cur = self.al_vel * self.vel_cur + self.al_vel_sf1 * self.vel_prev

        # TODO: compute self.vel_ctl.dt and self.vel_ctl.half_dt

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

        # TODO: compute self.steer_ctl.dt and self.steer_ctl.half_dt OR put in getSteerDes ??

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

    def low_control_inf( self ):
        while True:

            # velocity control

            # compute error
            self.vel_ctl.err_prev = self.vel_ctl.err
            self.vel_ctl.err = self.vel_des_cur - self.vel_cur

            vel_com_out = self.vel_ctl.compute_pid()

            self.vel_pub( vel_com_out )

            # ======================================

            # steering control
            self.steer_ctl.err_prev = self.steer_ctl.err
            self.steer_ctl.err = self.steer_des_cur - self.steer_cur

            steer_com_out = self.steer_ctl.compute_pid()

            self.steer_pub( steer_com_out )

        #
    #

#


# ======================================
# ======================================

if __name__ == 'main':
    rospy.init_node("lowlevel_control_oo", anonymous=False)
    control = LowLevelControl()
    rospy.spin()
    #
#

















#
