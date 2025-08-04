import time

class PID:
    """
    Cascading PID controller
    """
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_time = None
        self.integral = 0.0
        self.prev_error = 0.0

    def get_values(self):
        """
        Returns the current PID values
        :return: Tuple of (kp, ki, kd)
        """
        return self.kp, self.ki, self.kd

    def modify_values(self, kp, ki, kd):
        """
        Allows users to modify the PID values
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, setpoint, measured_value):
        """
        Get the cascading PID output
        :param setpoint: Desired value
        :param measured_value: Current value
        :param dt: Time difference since last update
        :return: PID output
        """
        curr_time = time.time()
        if self.prev_time is None:
            dt = 0.01
        else:
            dt = curr_time - self.prev_time
        
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output
