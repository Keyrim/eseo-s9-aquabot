

class PidController:
    def __init__(self, kp, ki, kd, max_u=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e = 0          # Error
        self.e_sum = 0      # Sum of errors
        self.e_prev = 0     # Previous error
        self.u = 0          # Control signal
        self.max_u = max_u  # Maximum control signal

    def compute(self, e, dt):
        self.e = e
        self.e_sum += e*dt
        self.u = self.kp*e #+ self.ki*self.e_sum + self.kd*(e - self.e_prev)/dt
        if self.max_u != 0:
            self.u = min(self.u, self.max_u)
        self.e_prev = e
        return self.u