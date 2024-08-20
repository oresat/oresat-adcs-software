from numpy.random import default_rng
import numpy as np
from ..functions import quaternion, vector

rng = default_rng()


class Sensor():
    '''Basic model of a sensor. Accesses a state of the satellite and possibly adds noise.
    Maybe add a normalize vector class to automatically make the vector a unit vector

    Parameters
        mean : float : Mean value of noise (generally 0).
        std_dev : float : Standard deviation of noise.
        env : environment 
        size : int : the length of the vector or array
    '''
    def __init__(self, mean, std_dev, env, size=3):
        self.mean = mean
        self.std_dev = std_dev
        self.environment = env
        self.size = size


    def generate_noise(self, noisy):
        '''Noise to add if noisy
        
        Parameters
            noisy (bool): True to have noise, False to return zeros
            
        Returns
            numpy.ndarray: Array of noise to add
        '''
        return np.zeros(self.size) if not noisy else rng.normal(self.mean, self.std_dev, self.size)


    def true_value(self, state):
        '''Actual value of state.

        Parameters
            state : dynamics.SatelliteState : state of satellite to measure

        Returns
            numpy.ndarray : Actual value of state.
        '''
        return np.zeros(self.size)
    

    def measurement(self, state, noisy):
        '''Possibly noisy measurement of state.

        Parameters
            state (dynamics.SatelliteState) state of satellite to measure
            noisy (bool) True if random noise should be added to measurement.

        Returns
            numpy.ndarray : Possibly noisy measurement of state.
        '''
        return self.true_value(state) + self.generate_noise(noisy) 




# Linear orbital states
class GPS_pos(Sensor):
    '''GPS position measurement.

    Parameters
        mean : float : Mean value of noise (generally 0).
        std_dev : float : Standard deviation of noise.
        env : environment 
        size : int : the length of the vector or array
    '''

    def true_value(self, state):
        '''Actual value of state.

        Returns
            numpy.ndarray : Actual value of state.
        '''
        return state[0]


class GPS_vel(Sensor):
    '''GPS velocity measurement.
    
    Parameters
        mean (float) : Mean value of noise (generally 0).
        std_dev (float) : Standard deviation of noise.
        env (environment) : environment 
        size (int): the length of the vector or array
    '''

    def true_value(self, state):
        '''Actual value of state.

        Returns
            numpy.ndarray : Actual value of state.
        '''
        return state[1]





# Attitude States
class StarTracker(Sensor):
    '''Model of star tracker, assuming star tracker returns quaternion measurements.
    Might need to adjust the standard deviation by 0.5 to convert from Euler angle error to quaternion error.
    Check with noise: https://github.com/oresat/oresat-star-tracker-software/blob/master/tests/test_lost.py

    Parameters
        mean : float : Mean value of noise (generally 0).
        std_dev : float : Standard deviation of noise.
        env : environment 
        size : int : the length of the vector or array
    '''

    def generate_noise(self, noisy):
        noise_v = np.zeros(self.size-1) if not noisy else rng.normal(self.mean, self.std_dev, self.size-1)
        return np.array([0, *noise_v])
    
    
    def true_value(self, state):
        '''Actual value of state.

        Returns
            numpy.ndarray : Actual value of state.
        '''
        return state[2]




class Gyro(Sensor):
    '''Discrete model of gyroscopes in IMU including bias drifting.
    From eq 4.54 in Markely & Crassidis, pg 147.

    Parameters
    arw_mean (float): Mean value of measurement noise (generally 0).
    arw_std_dev (float): Standard deviation of measurement noise.
    rrw_mean (float): Mean value of bias drift (generally 0).
    rrw_std_dev (float): Standard deviation of bias drift.
    init_bias (float): Standard deviation of initial bias.
    environment (env): Truth model of satellite.
    '''
    def __init__(self, arw_mean, arw_std_dev, rrw_mean, rrw_std_dev, init_bias, env):
        super().__init__(arw_mean, arw_std_dev, env)
        self.rrw_mean, self.rrw_std_dev = rrw_mean, rrw_std_dev
        self.bias                       = np.random.normal(0, init_bias, 3)
        self.last_bias                  = np.zeros(3)
        self.noise_coefficient          = 0
        #self.w                          = self.true_value()
        self.arw_var                    = arw_std_dev**2
        self.rrw_var                    = rrw_std_dev**2 / 12

    def propagate(self, dt):
        '''A gyroscope is itself a dynamical system, driven by what amounts to random noise.
        This propagates the gyros to the next time step.

        Parameters
            dt (float): Size of step to take in seconds.
        '''
        self.last_bias = self.bias
        self.new_bias  = self.bias + np.sqrt(dt) * np.random.normal(self.rrw_mean, self.rrw_std_dev, 3)
        self.noise_coefficient = np.sqrt(self.arw_var / dt + self.rrw_var * dt)
        # self.w         = self.true_value() + 0.5 * (self.new_bias + self.last_bias) + np.sqrt(self.arw_var / dt + self.rrw_var * dt) * np.random.normal(0, 1, 3)
        self.bias      = self.new_bias

    def generate_noise(self, noisy):
        '''Returns randomly generated noise based on how the error and bias have propogated
        
        Returns
            numpy.ndarray: array for noise to add    
        '''
        return np.zeros(self.size) if not noisy else (0.5 * (self.bias + self.last_bias) + self.noise_coefficient* np.random.normal(0, 1, 3))

    def true_value(self, state):
        '''Actual value of state.

        Returns
            numpy.ndarray : Actual value of state.
        '''
        return state[3] # could be aliased with state,.body_ang_vel



# Actuator States
class Wheel_vel(Sensor):
    '''Measurement of wheel velocities.
    
    Parameters
        mean : float : Mean value of noise (generally 0).
        std_dev : float : Standard deviation of noise.
        env : environment 
        size : int : the length of the vector or array
    '''

    def true_value(self, state):
        '''Actual value of state.

        Returns
            numpy.ndarray : Actual value of state.
        '''
        return state[4]


        
# Environment Sensors
# these still need the state for the attitude
class Magnetometer(Sensor):
    '''Model of magnetometers.

    Parameters
        mean : float : Mean value of noise (generally 0).
        std_dev : float : Standard deviation of noise.
        model : dynamic.DynamicalSystem : Truth model of satellite.
        size : int : the length of the vector or array
    '''
    
    def true_value(self, state):
        '''Actual value of magnetic field in body-coordinates.

        Returns
            numpy.ndarray : Actual value of magnetic field in body-coordinates.
        '''
        B = self.environment.magnetic_field(state)
        B_body = quaternion.sandwich(state[2], B)
        return B_body



# normalized vector
class SunSensor(Sensor):
    '''Model of sun sensors. More for convenience, the satellite won't have sun sensors.

    Parameters
        mean : float : Mean value of noise (generally 0).
        std_dev : float : Standard deviation of noise.
        model : dynamic.DynamicalSystem : Truth model of satellite.
        size : int : the length of the vector or array
    '''
    
    def true_value(self, state):
        '''Actual value of state.

        Returns
            numpy.ndarray : Actual value of state.
        '''
        S_inertial = self.environment.SRP_info(state)[2]
        S_body     = quaternion.sandwich(state[2], S_inertial)
        return S_body
