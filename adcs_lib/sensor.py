from numpy.random import default_rng
import numpy as np
import quaternion

rng = default_rng()

class Sensor():

    def __init__(self, mean, std_dev, noisy, model):
        self.mean = mean
        self.std_dev = std_dev
        self.noisy = noisy
        self.model = model

    def true_value(self):
        return np.zeros(3)

    def measurement(self):
        noise = np.zeros(3) if not self.noisy else rng.normal(self.mean, self.std_dev, 3)
        return self.true_value() + noise

class Magnetometer(Sensor):

    def true_value(self):
        r_ecef = self.model.GCI_to_ECEF.dot(self.model.state[0])
        length = np.linalg.norm(self.model.state[0])
        B = self.model.enviro.magnetic_field(r_ecef, length, self.model.GCI_to_ECEF)
        B_body = quaternion.sandwich(self.model.state[2], B)
        return B_body

class Gyro(Sensor):

    def true_value(self):
        return self.model.state[3]

class StarTracker(Sensor):

    def true_value(self):
        return self.model.state[2]

    def measurement(self):
        noise = np.zeros(4) if not self.noisy else rng.normal(self.mean, self.std_dev, 4)
        return quaternion.normalize(self.true_value() + noise)

class GPS_pos(Sensor):

    def true_value(self):
        return self.model.state[0]

class GPS_vel(Sensor):

    def true_value(self):
        return self.model.state[1]
