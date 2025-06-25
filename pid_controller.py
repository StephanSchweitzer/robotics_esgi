class PIDController:
    def __init__(self, kp=0.3, ki=0.01, kd=0.1, integral_limit=1000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        
        self.integral_error = 0
        self.previous_error = 0


    
    def compute(self, error):
        proportional = self.kp * error
        
        self.integral_error += error
        if abs(self.integral_error) > self.integral_limit:
            self.integral_error = self.integral_limit if self.integral_error > 0 else -self.integral_limit
        integral = self.ki * self.integral_error
        
        derivative_error = error - self.previous_error
        derivative = self.kd * derivative_error
        
        output = proportional + integral + derivative
        
        self.previous_error = error
        
        return output
    

    
    def reset(self):
        self.integral_error = 0
        self.previous_error = 0
    


    def tune(self, kp=None, ki=None, kd=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd