import numpy as np
from numpy.random import default_rng
from adcs_manager.adcs_lib import sensor, dynamic

rng = default_rng()

class KalmanState(dynamic.DynamicalSystem):

    def __init__(self, atd_bias, gyro_bias, covariance, position, lin_vel, attitude, body_ang_vel, wheel_vel, date_and_time):
        super().__init__(position, lin_vel, attitude, body_ang_vel, wheel_vel, date_and_time)
        self.state = np.array([atd_bias, gyro_bias, covariance, *self.state])
        self.GYRO_SIGMA = 5.818 * 10**(-4) # angular random walk, rad/sqrt(s), from datasheet
        self.DRIFT_SIGMA = 1.713 * 10**(-8) # rate random walk, rad / s^(3/2), guestimate
        self.BIAS_SIGMA = 0.05236 # rad/s initial bias std dev, guestimate
        self.SENSOR_SIGMA = 1.5 * 10**(-5) # rad, star tracker measurement noise, guestimate
        g = self.GYRO_SIGMA**2
        d = self.DRIFT_SIGMA**2
        # untuned process noise model, assuming variance is independent
        self.Q = np.array([
                [g, 0, 0, 0, 0, 0],
                [0, g, 0, 0, 0, 0],
                [0, 0, g, 0, 0, 0],
                [0, 0, 0, d, 0, 0],
                [0, 0, 0, 0, d, 0],
                [0, 0, 0, 0, 0, d]
                ])
        # change in linear EKF with respect to noise
        self.G = np.array([
                [-1, 0, 0, 0, 0, 0],
                [0, -1, 0, 0, 0, 0],
                [0, 0, -1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1.]
                ])
        # measurement covariance
        self.R = np.diag([self.SENSOR_SIGMA**2 for i in range(3)])

    # change in linear EKF wrt state estimate
    # w is estimated angular rate
    def F_mtrx(self, w):
        return np.array([
            [0,     w[2],  -w[1], -1, 0, 0],
            [-w[2], 0,     w[0],  0, -1, 0],
            [w[1],  -w[0], 0,     0, 0, -1],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]
        ])

    def vector_field(self, atd_bias, gyro_bias, covariance,
                     position, lin_vel, attitude, body_ang_vel, wheel_vel,
                     body_ang_accl, mag_moment, whl_accl):
        derivs = super().vector_field(position, lin_vel, attitude, body_ang_vel, wheel_vel, body_ang_accl, mag_moment, whl_accl)
        # farrenkopf's gyro dynamics error model
        w_est = body_ang_vel - gyro_bias - rng.normal(0, self.GYRO_SIGMA, 3)
        F = self.F_mtrx(w_est)

        dadt = np.cross(atd_bias, w_est) + gyro_bias
        dbdt = rng.normal(0, self.DRIFT_SIGMA, 3)
        # Riccati equation, propagation of kalman covariance matrix
        dPdt = F.dot(covariance) + covariance.dot(F.T) + self.G.dot(self.Q.dot(self.G.T))
        return np.array([dadt, dbdt, dPdt, derivs[0], derivs[1], derivs[2], derivs[3], derivs[4]], dtype=object)

class DummyFilter():
    def __init__(self, model):
        self.model = model
        self.magnetometer = sensor.Magnetometer(0, 0.05e-6, False, self.model)

    def output(self):
        return self.model.state, self.magnetometer.measurement()

    def update(self, sensor_data):
        pass

    def propagate(self, duration, zero_order_hold):
        pass
