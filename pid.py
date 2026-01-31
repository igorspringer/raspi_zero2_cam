# pid.py

class PID:
    def __init__(self, kp, ki, kd, limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.prev = 0
        self.sum = 0

    def update(self, error):
        self.sum += error
        d = error - self.prev
        self.prev = error

        out = self.kp * error + self.ki * self.sum + self.kd * d

        if out > self.limit:
            out = self.limit
        elif out < -self.limit:
            out = -self.limit

        return out
