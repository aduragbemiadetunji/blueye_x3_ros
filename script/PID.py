import time

class PID:
    def __init__(self, setpoint, Kp=0, Ki=0, Kd=0, sample_time=0.1):
        self.setpoint = setpoint
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time
        self.last_time = time.time()
        self.last_error = 0
        self.integral = 0

    def compute(self, feedback_value):
        current_time = time.time()
        elapsed_time = (current_time - self.last_time)*1000000
        error = self.setpoint - feedback_value
        self.integral += error * elapsed_time
        derivative = (error - self.last_error) / elapsed_time
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.last_time = current_time
        self.last_error = error
        return output