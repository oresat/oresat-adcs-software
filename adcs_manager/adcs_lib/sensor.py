from numpy.random import default_rng
import numpy as np
from adcs_lib import quaternion

rng = default_rng()

class Sensor():
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self, mean, std_dev, model):
        self.mean = mean
        self.std_dev = std_dev
        self.model = model

    def true_value(self):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return np.zeros(3)

    def measurement(self, noisy):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        noise = np.zeros(3) if not noisy else rng.normal(self.mean, self.std_dev, 3)
        return self.true_value() + noise

class Magnetometer(Sensor):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def true_value(self):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        r_ecef = self.model.GCI_to_ECEF.dot(self.model.state[0])
        length = np.linalg.norm(self.model.state[0])
        B = self.model.enviro.magnetic_field(r_ecef, length, self.model.GCI_to_ECEF)
        B_body = quaternion.sandwich(self.model.state[2], B)
        return B_body


class SunSensor(Sensor):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def true_value(self):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        S_inertial = self.model.enviro.sun_vector(self.model.clock, self.model.state[0])
        S_body     = quaternion.sandwich(self.model.state[2], S_inertial)
        return S_body

    def measurement(self, noisy):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        noise = np.zeros(3) if not noisy else rng.normal(self.mean, self.std_dev, 3)
        return quaternion.normalize(self.true_value() + noise)

class Gyro(Sensor):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self, arw_mean, arw_std_dev, rrw_mean, rrw_std_dev, init_bias, model):
        super().__init__(arw_mean, arw_std_dev, model)
        self.rrw_mean, self.rrw_std_dev = rrw_mean, rrw_std_dev
        self.bias = np.random.normal(0, init_bias, 3)

    def propagate(self, dt):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        self.bias += dt * np.random.normal(self.rrw_mean, self.rrw_std_dev, 3)

    def true_value(self):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return self.model.state[3]

    def measurement(self, noisy):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        noise = np.zeros(3) if not noisy else (self.bias + rng.normal(self.mean, self.std_dev, 3))
        return self.true_value() + noise

class StarTracker(Sensor):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def true_value(self):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return self.model.state[2]

    def measurement(self, noisy):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        noise_v = np.zeros(3) if not noisy else rng.normal(self.mean, self.std_dev, 3)
        noise = np.array([0, *noise_v])
        return quaternion.normalize(self.true_value() + noise)

class GPS_pos(Sensor):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def true_value(self):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return self.model.state[0]

class GPS_vel(Sensor):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def true_value(self):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return self.model.state[1]

class Wheel_vel(Sensor):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def true_value(self):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return self.model.state[4]

    def measurement(self, noisy):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        noise = np.zeros(4) if not noisy else rng.normal(self.mean, self.std_dev, 4)
        return self.true_value() + noise
