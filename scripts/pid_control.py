class pid():
    def __init__(self, kP, kI, kD, integral_bound = 300):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.value = 0
        self.last_error = 0
        self.integral = 0
        self.integral_bound = integral_bound
        self.error = 0
        self.derivative = 0
        self.output_limit = 7

    def set_state(self, c):
        self.value = c
    
    def get_output(self):
        return min(self.kP*self.error+self.kI*self.integral+self.kD*self.derivative, self.output_limit)
    
    def get_output_ff(self, kF, ff):
        return self.kP*self.error+self.kI*self.integral+self.kD*self.derivative+kF*ff
    
    def update(self, target, dt):
        self.error = target - self.value
        if self.integral+self.error*dt < self.integral_bound or self.integral > -self.integral_bound:
            self.integral = self.integral + self.error*dt
        self.integral = self.integral + self.error*dt
        if self.integral > self.integral_bound:
            self.integral = self.integral_bound
        elif self.integral < -self.integral_bound:
            self.integral = -self.integral_bound
        self.derivative = (self.error - self.last_error)*dt
        self.last_error = self.error

    
    def reset(self):
        self.integral = 0
        self.last_error = 0
        self.value = 0
    
