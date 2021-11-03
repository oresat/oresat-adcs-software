from numpy.random import default_rng
import numpy as np
from adcs_lib import quaternion, vector

rng = default_rng()

class Sensor():
    '''
    Basic model of a sensor. Accesses a state of the satellite and possibly adds noise.

    Parameters
    ----------
    mean : float
        Mean value of noise (generally 0).
    std_dev : float
        Standard deviation of noise.
    model : dynamic.DynamicalSystem
        Truth model of satellite.
    '''
    def __init__(self, mean, std_dev, model):
        self.mean = mean
        self.std_dev = std_dev
        self.model = model

    def true_value(self):
        '''
        Actual value of state.

        Returns
        -------
        numpy.ndarray
            Actual value of state.
        '''
        return np.zeros(3)

    def measurement(self, noisy):
        '''
        Possibly noisy measurement of state.

        Parameters
        ----------
        noisy : bool
            True if random noise should be added to measurement.

        Returns
        -------
        numpy.ndarray
            Possibly noisy measurement of state.
        '''
        noise = np.zeros(3) if not noisy else rng.normal(self.mean, self.std_dev, 3)
        return self.true_value() + noise

class Magnetometer(Sensor):
    '''
    Model of magnetometers.

    Parameters
    ----------
    mean : float
        Mean value of noise (generally 0).
    std_dev : float
        Standard deviation of noise.
    model : dynamic.DynamicalSystem
        Truth model of satellite.
    '''
    def true_value(self):
        '''
        Actual value of magnetic field in body-coordinates.

        Returns
        -------
        numpy.ndarray
            Actual value of magnetic field in body-coordinates.
        '''
        r_ecef = self.model.GCI_to_ECEF.dot(self.model.state[0])
        length = np.linalg.norm(self.model.state[0])
        B = self.model.enviro.magnetic_field(r_ecef, length, self.model.GCI_to_ECEF)
        B_body = quaternion.sandwich(self.model.state[2], B)
        return B_body

class SunSensor(Sensor):
    '''
    Model of sun sensors. More for convenience, the satellite won't have sun sensors.

    Parameters
    ----------
    mean : float
        Mean value of noise (generally 0).
    std_dev : float
        Standard deviation of noise.
    model : dynamic.DynamicalSystem
        Truth model of satellite.
    '''
    def true_value(self):
        '''
        Actual value of state.

        Returns
        -------
        numpy.ndarray
            Actual value of state.
        '''
        S_inertial = self.model.enviro.sun_vector(self.model.clock, self.model.state[0], self.model.state[2])[0]
        S_body     = quaternion.sandwich(self.model.state[2], S_inertial)
        return S_body

    def measurement(self, noisy):
        '''
        Since the sun vector just represents direction, we normalize it after adding noise.

        Parameters
        ----------
        noisy : bool
            True if random noise should be added to measurement.

        Returns
        -------
        numpy.ndarray
            Possibly noisy measurement of state.
        '''
        noise = np.zeros(3) if not noisy else rng.normal(self.mean, self.std_dev, 3)
        return vector.normalize(self.true_value() + noise)

class Gyro(Sensor):
    '''
    Discrete model of gyroscopes in IMU including bias drifting.
    From eq 4.54 in Markely & Crassidis, pg 147.

    Parameters
    ----------
    arw_mean : float
        Mean value of measurement noise (generally 0).
    arw_std_dev : float
        Standard deviation of measurement noise.
    rrw_mean : float
        Mean value of bias drift (generally 0).
    rrw_std_dev : float
        Standard deviation of bias drift.
    init_bias : float
        Standard deviation of initial bias.
    model : dynamic.DynamicalSystem
        Truth model of satellite.
    '''
    def __init__(self, arw_mean, arw_std_dev, rrw_mean, rrw_std_dev, init_bias, model):
        super().__init__(arw_mean, arw_std_dev, model)
        self.rrw_mean, self.rrw_std_dev = rrw_mean, rrw_std_dev
        self.bias                       = np.random.normal(0, init_bias, 3)
        self.last_bias                  = np.zeros(3)
        self.w                          = self.true_value()
        self.arw_var                    = arw_std_dev**2
        self.rrw_var                    = rrw_std_dev**2 / 12

    def propagate(self, dt):
        '''
        A gyroscope is itself a dynamical system, driven by what amounts to random noise.
        This propagates the gyros to the next time step.

        Parameters
        ----------
        dt : float
            Size of step to take in seconds.
        '''
        self.last_bias = self.bias
        self.new_bias  = self.bias + np.sqrt(dt) * np.random.normal(self.rrw_mean, self.rrw_std_dev, 3)
        self.w         = self.true_value() + 0.5 * (self.new_bias + self.last_bias) + np.sqrt(self.arw_var / dt + self.rrw_var * dt) * np.random.normal(0, 1, 3)
        self.bias      = self.new_bias

    def true_value(self):
        '''
        Actual value of state.

        Returns
        -------
        numpy.ndarray
            Actual value of state.
        '''
        return self.model.state[3]

    def measurement(self, noisy):
        '''
        If noisy measurements are used, accounts for both drift bias and measurement noise.

        Parameters
        ----------
        noisy : bool
            True if random noise should be added to measurement.

        Returns
        -------
        numpy.ndarray
            Possibly noisy measurement of state.
        '''
        value = self.w if noisy else self.true_value()
        return value

class StarTracker(Sensor):
    '''
    Model of star tracker, assuming star tracker returns quaternion measurements.
    Might need to adjust the standard deviation by 0.5 to convert from Euler angle error to quaternion error.

    Parameters
    ----------
    mean : float
        Mean value of noise (generally 0).
    std_dev : float
        Standard deviation of noise.
    model : dynamic.DynamicalSystem
        Truth model of satellite.
    '''
    def true_value(self):
        '''
        Actual value of state.

        Returns
        -------
        numpy.ndarray
            Actual value of state.
        '''
        return self.model.state[2]

    def measurement(self, noisy):
        '''
        Attitude measurement. If noisy, we only inject noise into the vector part of the quaternion, then renormalize.

        Parameters
        ----------
        noisy : bool
            True if random noise should be added to measurement.

        Returns
        -------
        numpy.ndarray
            Possibly noisy measurement of state.
        '''
        noise_v = np.zeros(3) if not noisy else rng.normal(self.mean, self.std_dev, 3)
        noise = np.array([0, *noise_v])
        return vector.normalize(self.true_value() + noise)

class GPS_pos(Sensor):
    '''
    GPS position measurement.

    Parameters
    ----------
    mean : float
        Mean value of noise (generally 0).
    std_dev : float
        Standard deviation of noise.
    model : dynamic.DynamicalSystem
        Truth model of satellite.
    '''
    def true_value(self):
        '''
        Actual value of state.

        Returns
        -------
        numpy.ndarray
            Actual value of state.
        '''
        return self.model.state[0]

class GPS_vel(Sensor):
    '''
    GPS velocity measurement.

    Parameters
    ----------
    mean : float
        Mean value of noise (generally 0).
    std_dev : float
        Standard deviation of noise.
    model : dynamic.DynamicalSystem
        Truth model of satellite.
    '''
    def true_value(self):
        '''
        Actual value of state.

        Returns
        -------
        numpy.ndarray
            Actual value of state.
        '''
        return self.model.state[1]

class Wheel_vel(Sensor):
    '''
    Measurement of wheel velocities.

    Parameters
    ----------
    mean : float
        Mean value of noise (generally 0).
    std_dev : float
        Standard deviation of noise.
    model : dynamic.DynamicalSystem
        Truth model of satellite.
    '''
    def true_value(self):
        '''

        Actual value of state.

        Returns
        -------
        numpy.ndarray
            Actual value of state.
        '''
        return self.model.state[4]

    def measurement(self, noisy):
        '''
        Measures the four wheel velocities.

        Parameters
        ----------
        noisy : bool
            True if random noise should be added to measurement.

        Returns
        -------
        numpy.ndarray
            Possibly noisy measurement of state.
        '''
        noise = np.zeros(4) if not noisy else rng.normal(self.mean, self.std_dev, 4)
        return self.true_value() + noise
