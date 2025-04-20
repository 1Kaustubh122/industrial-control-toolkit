"""
PID Controller

Gain Params:
    kp = Proportional constant
    kd = Derivative constant
    ki = Integral constant

Other Params:
    dt = Discrete time step
    output_limits = (min_output, max_output)
    anti_windup = bool, enbles/disables integral clamping 
                -> Prevents integral from overshooting from accumulated error
    derivative_filter = low pass filter coefficient (0.0 = No Filter, 1 = Full Filter) 
                        ->  Very sensitive to noise,
                            small noise spike in measurments can cause huge
                            jumps in derivative value, so we smooth it with a filter.
"""
class PID:
    def __init__(self, kp, kd, ki, dt, output_limits=(None, None), anti_windup=True, derivative_filter=0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        
        self.min_output, self.max_output = output_limits
        self.anti_windup = anti_windup
        self.derivative_filter = derivative_filter
        
        self.reset()
    
    def reset(self):
        self.integral = 0.0                         
        self.last_D = 0.0                         # Previous Derivative
        self.last_error = 0.0
        self.last_output = 0.0
        
    def compute(self, set_point, measurement):
        error = set_point - measurement
        
        # Proportiional Term
        P = self.kp * error
        
        # Integral Term
        self.integral += error * self.dt
        I = self.ki * self.integral
        
        # Derivative term with filtering
        raw_derivative = (error - self.last_error) / self.dt
        D = self.kd * ((1 - self.derivative_filter) * raw_derivative + self.derivative_filter * self.last_D)
        
        # PID output
        output = P + I + D
        
        # Applyin output limits
        if self.min_output is not None:
            output = max(self.min_output, output)
        if self.max_output is not None:
            output = min(self.max_output, output)
        
        # Anti-widup correction
        if self.anti_windup:
            unclipped_output = P+I+D 
            if output != unclipped_output:
                self.integral -= error * self.dt
                
        # Storing the value for next itr
        self.last_error = error
        self.last_D = raw_derivative
        self.last_output = output
        
        return output