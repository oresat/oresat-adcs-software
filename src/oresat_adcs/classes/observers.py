import numpy as np
from numpy.random import default_rng

from ..functions import quaternion, vector
from . import dynamics

# attitude estimation may change locations
from ..functions import attitude_estimation

class DiscreteAttitudeStartrackerKalman():
    '''
    Discrete-time Multiplicative Extended Kalman Filter for attitude estimation.
    Estimates attitude error and gyroscope bias.
    Takes measurements as quaternions from a Startracker.
    Refer to Markely & Crassidis for details.

    Parameters
    ----------
    q_initial : numpy.ndarray
        Initial attitude estimate.
    '''
    def __init__(self, q_initial):
        self.GYRO_VAR     = (2.79e-4)**2 # angular random walk, rad/s/sqrt(Hz), https://jpieper.com/2020/03/09/bringing-up-the-imu-on-the-pi3-hat/
        self.DRIFT_VAR    = (8.73e-7)**2 # rate random walk, same website
        self.INITIAL_BIAS = 3.15e-5 # bias stability, same website

        #self.GYRO_SIGMA   = 0.00174533 # angular random walk, rad/s, from datasheet
        #self.DRIFT_SIGMA  = 1.9391e-5 # rad/s^2, assuming 200 C/s temp change #1.713e-8 # rate random walk, rad / s^(3/2), guestimate
        #self.INITIAL_BIAS = 0.0174533 # rad/s, from datasheet (1 deg/s)
        self.SENSOR_SIGMA = 0.75e-7 # rad, star tracker measurement noise, guestimate
        #self.MSENSOR_SIGMA = 4e-8 # rad, magnetometer measurement noise

        self.dq = np.zeros(3)
        self.db = np.zeros(3)
        self.P  = np.diag([self.SENSOR_SIGMA**2, self.SENSOR_SIGMA**2, self.SENSOR_SIGMA**2,
                            self.INITIAL_BIAS**2, self.INITIAL_BIAS**2, self.INITIAL_BIAS**2])
        # quaternion
        self.q = q_initial
        # gyroscope bias
        self.b = np.zeros(3)

        # change in linear EKF with respect to noise
        self.G = np.diag([-1, -1, -1, 1, 1, 1])

        # measurement covariance
        self.R = np.diag([self.SENSOR_SIGMA**2 for i in range(3)])

    def measurement(self, q_ref):
        '''
        Measurement update for Kalman state.

        Parameters
        ----------
        q_ref : numpy.ndarray
            Attitude measurement.
        '''
        self.K  = np.hsplit(self.P, 2)[0].dot(np.linalg.inv(self.R + np.vsplit(np.hsplit(self.P, 2)[0], 2)[0]))
        temp    = np.eye(6) - np.block([self.K, np.zeros((6,3))])
        self.P  = temp.dot(self.P) # investigate the claim that the expression below is more numerically stable
        #self.P  = temp.dot(self.P.dot(temp.T)) + self.K.dot(self.R.dot(self.K.T))
        dx      = self.K.dot(quaternion.error_quat(q_ref, self.q)[1:] - self.dq)
        self.dq = dx[:3]
        self.db = dx[3:]
        self.reset()

    def reset(self):
        '''
        Reset step for Kalman state.
        '''
        self.q  = vector.normalize(self.q + quaternion.xi(self.q).dot(self.dq) * 0.5)
        self.dq = np.zeros(3)
        self.b += self.db
        self.db = np.zeros(3)

    def state_transition(self, w, dt):
        '''
        Calculates state transition matrices for discrete propagation of attitude and covariance.

        Parameters
        ----------
        w : numpy.ndarray
            Angular velocity estimate from gyroscopes.
        dt : float
            Time (s) to step forward by.

        Returns
        -------
        numpy.ndarray
            Process noise covariance.
        numpy.ndarray
            Covariance propagation matrix.
        numpy.ndarray
            Attitude state transition matrix.
        '''
        Q11 = self.GYRO_VAR * dt + 0.3333333 * self.DRIFT_VAR * dt**3
        Q12 = - 0.5 * self.DRIFT_VAR * dt**2
        Q22 = self.DRIFT_VAR * dt
        # untuned process noise model, assuming variance is independent
        Q   = np.block([[Q11 * np.eye(3), Q12 * np.eye(3)],
                        [Q12 * np.eye(3), Q22 * np.eye(3)]])

        w_X      = vector.cross(w)
        w_X2     = w_X.dot(w_X)
        norm_w   = np.linalg.norm(w)
        w_square = norm_w**2
        w_cube   = norm_w**3
        s        = np.sin(norm_w * dt)
        c        = np.cos(norm_w * dt)

        if w_square != 0:
            phi_11 = np.eye(3) - s / norm_w * w_X + (1 - c) / w_square * w_X2
            if w_cube != 0:
                phi_12 = -np.eye(3) * dt + (1 - c) / w_square * w_X - (norm_w * dt - s) / w_cube * w_X2
            else:
                phi_12 = -np.eye(3) * dt + (1 - c) / w_square * w_X
        else:
            if norm_w != 0:
                phi_11 = np.eye(3) - s / norm_w * w_X
            else:
                phi_11 = np.eye(3)
            phi_12 = -np.eye(3) * dt

        phi      = np.block([
                    [phi_11, phi_12],
                    [np.zeros((3,3)), np.eye(3)]
                    ])

        s        = np.sin(norm_w * dt * 0.5)
        c        = np.cos(norm_w * dt * 0.5)

        if norm_w != 0:
            psi = s / norm_w * w
        else:
            psi = np.zeros(3)

        omegaleft  = np.array([[c, *psi]]).T
        omegaright = np.block([[-psi],
                                [np.eye(3) - vector.cross(psi)]])
        omega      = np.block([omegaleft, omegaright])

        return Q, phi, omega

    def propagate(self, w_sensor, dt):
        '''
        A priori estimate of state.

        Parameters
        ----------
        w_sensor : numpy.ndarray
            Angular velocity measurements, straight from gyroscopes.
        dt : float
            Length of time to step forward by.
        '''
        w             = w_sensor - self.b
        Q, phi, omega = self.state_transition(w, dt)
        GQG_T         = self.G.dot(Q.dot(self.G.T))
        self.P        = phi.dot(self.P.dot(phi.T)) + GQG_T
        self.q        = omega.dot(self.q)

    def output(self):
        '''
        Kalman state estimate.

        Returns
        -------
        numpy.ndarray
            Attitude estimate.
        numpy.ndarray
            Gyro bias estimate.
        '''
        return self.q, self.b








class DiscretePositionKalman():
    '''Discrete-time Extended Kalman Filter for position and velocity estimates.

    Parameters
        x : numpy.ndarray : Initial position estimate.
        v : numpy.ndarray : Initial velocity estimate.
        date_and_time : list : Initial date and time. (Y, M, D, h, m, s) format.
    '''
    def __init__(self, env, state):


        x_sigma = 10.0598 # m
        v_sigma = 0.52038 # m/s
        a_sigma = 0.018166 # m/s^2

        # system noise covariance
        a_var = a_sigma**2
        self.Q = np.diag([0, 0, 0, a_var, a_var, a_var])

        # measurement noise covariance
        x_var = x_sigma**2
        v_var = v_sigma**2
        self.R = np.diag([x_var, x_var, x_var, v_var, v_var, v_var])
        self.P = np.eye(6)

        # at a later point, calculate these parameters from the model, JIC
        self.a = 1.28e-6   # s^-2, mu/r^3 for circular orbit
        self.b = 1.618e-10 # s^-1, drag coeff assuming constant speed and density

        self.environment = env
        self.state = state

        # self.model = dynamics.ReducedDynamicalSystem(x, v, date_and_time)
        # self.x = np.block([self.model.state[0], self.model.state[1]])
        self.x = np.block([self.state.position, self.state.velocity])


    def estimate_attitude(self, B_ref, a_ref):
        '''Estimates the attitude of the satellite, given measurements from magnetometer and accelerometer.
        Note, the TRIAD algorithm uses two independent vectors, measured via sensors on the satellite.
        This are normally the magnetic field and either: the sun vector or nadir vector.

        If you don't have a sun sensor or earth sensor, then you should probably have a star tracker.
        And honestly, this should be a part of the attitude kalman filter, not the position/velocity one.

        Parameters
            B_ref : numpy.ndarray : Magnetic field (T) reading in body coordinates.
            a_ref : numpy.ndarray : Acceleration (m/s^2) reading in body coordinates.

        Returns
            numpy.ndarray or bool : Quaternion attitude or False if solver failed.
        '''
        #B_intl  = self.model.enviro.magnetic_field(self.x[:3], self.model.GCI_to_ECEF)
        #a_intl  = self.model.enviro.forces(self.x[:3], self.x[3:]) / self.model.satellite.mass
        #q       = attitude_estimation.triad_algorithm(vector.normalize(B_intl), vector.normalize(a_intl), vector.normalize(B_ref), vector.normalize(a_ref))
        #q       = attitude_estimation.flae([0.6, 0.4], [B_intl, a_intl], [B_ref, a_ref])
        #return q

        pass

    def F_continuous(self):
        '''Continuous time covariance propagation matrix.

        Returns
            numpy.ndarray : Continuous time covariance propagation matrix.
        '''
        F11      = np.zeros((3,3))
        F12      = np.eye(3)
        mu = -self.environment.MU
        r = np.linalg.norm(self.x[:3])
        r3term = -1/r**3
        r5term = 3/r**5
        x = self.x[0]
        y = self.x[1]
        z = self.x[2]
        grad_a_1 = mu * np.array([r3term + r5term * x**2, r5term * x * y, r5term * x * z])
        grad_a_2 = np.array([grad_a_1[1], mu * (r3term + r5term * y**2), mu * r5term * z * y])
        grad_a_3 = np.array([grad_a_1[2], grad_a_2[2], mu * (r3term + r5term * z**2)])
        F21      = np.block([[grad_a_1], [grad_a_2], [grad_a_3]])
        F22      = np.eye(3) * self.b #np.zeros((3,3)) # very rough approx of drag's impact on covariance for now, improve if we care
        F        = np.block([[F11, F12],
                             [F21, F22]])
        return F


    def state_transition(self, dt):
        '''Calculates state transition matrices for discrete propagation of position, velocity, and covariance.
        Neglects the contribution of Earth's rotating atmosphere to velocity. For more accuracy, use RK4 integration instead.

        Parameters
            dt : float : Time (s) to step forward by.

        Returns
            numpy.ndarray : Covariance propagation matrix.
            numpy.ndarray : State transition matrix.
        '''
        t2         = dt**2

        F          = self.F_continuous()
        F2         = F.dot(F)
        F_discrete = np.eye(6) + F * dt + 0.5 * F2 * t2# + 0.1666 * F.dot(F2) * dt**3 # H.O.T. not needed

        A11        = 1 - 0.5 * self.a * t2
        A12        = dt - 0.5 * self.b * t2 - 0.1666 * self.a * dt * t2
        A21        = - self.a * dt
        A22        = A11 - self.b * dt
        A_discrete = np.block([[A11*np.eye(3), A12*np.eye(3)],
                               [A21*np.eye(3), A22*np.eye(3)]])

        return F_discrete, A_discrete


    def update(self, x_ref, v_ref):
        '''Measurement update for Kalman state.

        Parameters
            x_ref : numpy.ndarray : Measured position.
            v_ref : numpy.ndarray : Measured velocity.
        '''
        y      = np.block([x_ref, v_ref])
        self.K = self.P.dot(np.linalg.inv(self.P + self.R))
        self.P = (np.eye(6) - self.K).dot(self.P)
        self.x = self.x + self.K.dot(y - self.x)

        # save the updated state to pass to the environmen
        self.state[0] = self.x[:3]
        self.state[1] = self.x[3:]
        self.state.update()


    def propagate(self, dt):
        '''A priori estimate of state.

        Parameters
            dt : float : Length of time to step forward by.
        '''
        F, A = self.state_transition(dt)
        self.P = F.dot(self.P.dot(F.T)) + self.Q
        self.x = A.dot(self.x)
        

    def output(self):
        '''Kalman state estimate.

        Returns
            numpy.ndarray : Position estimate.
            numpy.ndarray : Velocity estimate.
        '''
        return self.x[:3], self.x[3:]










class KalmanFilters():
    '''Wraps the two Kalman filters into one object.
    This probably isn't the best way to do this, but come back to that later.

    Parameters
        gyro_step_size : float : Time step for attitude propagation.
        gps_step_size : float : Time step for position propagation.
        truth_model : dynamic.DynamicalSystem : Source of measurements. Eventually this will be replaced by actual sensors.
    '''
    def __init__(self, gyro_step_size, gps_step_size, truth_model):
        self.truth_model = truth_model
        init_state = self.truth_model.measurement(noisy=True)
        x_init, v_init, q_init = init_state[:3]
        self.gyro_dt = gyro_step_size
        self.gps_dt  = gps_step_size
        self.PosFilter = DiscretePositionKalman(x_init, v_init, truth_model.init_date)
        self.AttFilter = DiscreteAttitudeStartrackerKalman(q_init)

    def output(self):
        '''Kalman state estimate.

        Returns
            numpy.ndarray : Array of arrays for estimated state, in usual order.
        '''
        state = self.truth_model.measurement(noisy=True)
        x, v  = self.PosFilter.output()
        q, b  = self.AttFilter.output()
        state[0] = x
        state[1] = v
        state[2] = q
        state[3] -= b
        self.last_w = state[3]
        #state = self.truth_model.measurement(noisy=False) # for when I want to test with perfect info
        return state

    def update(self, sensor_data):
        '''Measurement update for filters. Perhaps these should be detangled.

        Parameters
            sensor_data : numpy.ndarray : Array of arrays for measured state, in usual order.
        '''
        self.PosFilter.update(*sensor_data[:2])
        self.AttFilter.measurement(sensor_data[2])

    def propagate(self, duration):
        '''A priori estimate of state.

        Parameters
            duration : float : Length of time to propagate filters forward by.
        '''
        t = self.gyro_dt
        w = self.last_w # when this is asynch, this will be an actual gyro reading
        while t < duration or abs(t - duration) < 0.001:
            self.AttFilter.propagate(w, self.gyro_dt)
            t += self.gyro_dt
        t = self.gps_dt
        while t < duration or abs(t - duration) < 0.001:
            self.PosFilter.propagate(self.gps_dt)
            self.PosFilter.model.clock.tick(self.gps_dt)
            self.PosFilter.model.update_transformation_matrices()
            t += self.gps_dt



