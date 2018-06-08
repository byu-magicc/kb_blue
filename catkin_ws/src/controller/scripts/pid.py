class PID:
    def __init__(self, kp, kd, ki, min, max, tau=0.05):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.min = min
        self.max = max
        self.tau = tau

        self.derivative = 0.0
        self.integral = 0.0

        self.last_error = 0.0

    def run(self, error, dt, derivative=None):

        # P term
        if self.kp:
            p_term = self.kp * error
        else:
            p_term = 0.0

        # D term
        if self.kd:
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
        if self.ki:
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
        if self.ki:
            if abs(p_term + d_term) > abs(u_sat): # PD is already saturating, so set integrator to 0 but don't let it run backwards
                self.integral = 0
            else: # otherwise only let integral term at most take us just up to saturation
                self.integral = (u_sat - p_term - d_term) / self.ki

        # bookkeeping
        self.last_error = error

        return u_sat
