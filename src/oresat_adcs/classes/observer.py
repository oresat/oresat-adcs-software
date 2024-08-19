import numpy as np
from numpy.random import default_rng
from scipy import optimize

from ..functions import quaternion, vector
from ..classes import dynamics
from . import sensor


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
    def __init__(self, x, v, date_and_time):
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

        self.model = dynamics.ReducedDynamicalSystem(x, v, date_and_time)
        self.x = np.block([self.model.state[0], self.model.state[1]])

    def estimate_attitude(self, B_ref, a_ref):
        '''Estimates the attitude of the satellite, given measurements from magnetometer and accelerometer.

        Parameters
            B_ref : numpy.ndarray : Magnetic field (T) reading in body coordinates.
            a_ref : numpy.ndarray : Acceleration (m/s^2) reading in body coordinates.

        Returns
            numpy.ndarray or bool : Quaternion attitude or False if solver failed.
        '''
        B_intl  = self.model.enviro.magnetic_field(self.x[:3], self.model.GCI_to_ECEF)
        a_intl  = self.model.enviro.forces(self.x[:3], self.x[3:]) / self.model.satellite.mass
        q       = triad_algorithm(vector.normalize(B_intl), vector.normalize(a_intl), vector.normalize(B_ref), vector.normalize(a_ref))
        #q       = flae([0.6, 0.4], [B_intl, a_intl], [B_ref, a_ref])
        return q

    def F_continuous(self):
        '''Continuous time covariance propagation matrix.

        Returns
            numpy.ndarray : Continuous time covariance propagation matrix.
        '''
        F11      = np.zeros((3,3))
        F12      = np.eye(3)
        mu = -self.model.enviro.MU
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
        self.model.state = np.array([self.x[:3], self.x[3:]])

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









def triad_algorithm(r1, r2, b1, b2):
    '''Essentially the TRIAD algorithm, and technically suboptimal. For details refer to
    "Fast Quaternion Attitude Estimation from Two Vector Measurements" Markely 2002.

    Parameters
        r1 : numpy.ndarray : More accurate inertial estimate.
        r2 : numpy.ndarray : Less accurate inertial estimate.
        b1 : numpy.ndarray : More accurate body measurement.
        b2: numpy.ndarray : Less accurate body measurement.

    Returns
        numpy.ndarray or bool : Quaternion attitude or False if solver failed.
    '''
    # still not handling a couple degenerate cases very well...
    r3       = np.cross(r1, r2)
    b3       = np.cross(b1, b2)
    if r3 is np.zeros(3) or b3 is np.zeros(3):
        return False
    if b1 is -r1:
        return False

    dot_term = 1 + np.dot(b1, r1)
    mu       = dot_term * np.dot(b3, r3) - np.dot(b1, r3) * np.dot(r1, b3)
    sum_term = b1 + r1
    nu       = np.dot(sum_term, np.cross(b3, r3))
    rho      = np.sqrt(mu**2 + nu**2)
    if mu >= 0:
        rhoplusmu = rho + mu
        rhomudot  = rhoplusmu * dot_term
        mult   = 0.5 * 1 / np.sqrt(rho * rhomudot)
        v      = rhoplusmu * np.cross(b1, r1) + nu * sum_term
        q_part = np.array([rhomudot, *v])
        q      = mult * q_part
    else:
        rhominmu = rho - mu
        mult   = 0.5 * 1 / np.sqrt(rho * rhominmu * dot_term)
        v      = nu * np.cross(b1, r1) + rhominmu * sum_term
        q_part = np.array([nu * dot_term, *v])
        q      = mult * q_part
    return q

def flae(a, r, b):
    # could precompute constants
    # not actually any faster, i've been lied to
    def sum_across(a, r, b, n, i, j):
        return sum([a[k] * r[k][i] * b[k][j] for k in range(n)])

    def f(x, t1, t2, t3):
        return x**4 + t1 * x**2 + t2 * x + t3

    def f_p(x, t1, t2, t3):
        return 4 * x**3 + 2 * t1 * x + t2

    r_hat  = [vector.normalize(ri) for ri in r]
    b_hat  = [vector.normalize(bi) for bi in b]
    n      = len(a)

    Hx = np.array([sum_across(a, r_hat, b_hat, n, 0, j) for j in range(3)])
    Hy = np.array([sum_across(a, r_hat, b_hat, n, 1, j) for j in range(3)])
    Hz = np.array([sum_across(a, r_hat, b_hat, n, 2, j) for j in range(3)])

    W  = np.array([[Hx[0] + Hy[1] + Hz[2], -Hy[2] + Hz[1],        -Hz[0] + Hx[2],        -Hx[1] + Hy[0]],
                   [-Hy[2] + Hz[1],         Hx[0] - Hy[1] - Hz[2], Hx[1] + Hy[0],         Hx[2] + Hz[0]],
                   [-Hz[0] + Hx[2],         Hx[1] + Hy[0],         Hy[1] - Hx[0] - Hz[2], Hy[2] + Hz[1]],
                   [-Hx[1] + Hy[0],         Hx[2] + Hz[0],         Hy[2] + Hz[1],         Hz[2] - Hy[1] - Hx[0]]])

    tau1 = -2 * (np.dot(Hx, Hx) + np.dot(Hy, Hy) + np.dot(Hz, Hz))
    tau2 = 8 * np.dot(Hx, np.cross(Hz, Hy))
    #tau2 = 8 * (Hx[2]*Hy[1]*Hz[0] - Hx[1]*Hy[2]*Hz[0] - Hx[2]*Hy[0]*Hz[1] + Hx[0]*Hy[2]*Hz[1] + Hx[1]*Hy[0]*Hz[2] - Hx[0]*Hy[1]*Hz[2])
    tau3 = np.linalg.det(W)

    lam = optimize.newton(f, 1, fprime=f_p, args=(tau1, tau2, tau3), maxiter=4, rtol=0.00001)

    # tried and failed to get analytic method working
    '''tau1_sq = tau1**2
    tau1and3 = tau1_sq + 12 * tau3
    T0 = 2 * tau1**3 + tau2 * 27 * tau2 - 72 * tau1 * tau3
    print(T0**2, 4 * tau1and3**3)
    T1 = (T0 + np.sqrt(T0**2 - 4 * tau1and3**3))**0.3333333
    T2 = np.sqrt(2**0.666666 * T1 - 4 * tau1 + 2**1.3333333 * tau1and3 / T1)

    mult = 1 / (2 * np.sqrt(6))
    term1 = -T2**2
    term2 = -12 * tau1
    term3 = 12 * np.sqrt(6) * tau2 / T2

    lam = []
    lam.append(mult * (T2 - np.sqrt(term1 + term2 - term3)))
    lam.append(mult * (T2 + np.sqrt(term1 + term2 - term3)))
    lam.append(-mult * (T2 + np.sqrt(term1 + term2 + term3)))
    lam.append(-mult * (T2 - np.sqrt(term1 + term2 + term3)))

    dist = [abs(l - 1) for l in lam]
    i = np.argmin(dist)
    N = W - np.eye(4) * lam[i]'''

    N = W - np.eye(4) * lam

    pivot = N[0][0]
    N[0] /= pivot
    N[1] -= N[1][0]*N[0]
    N[2] -= N[2][0]*N[0]
    N[3] -= N[3][0]*N[0]

    pivot = N[1][1]
    N[1] /= pivot
    N[0] -= N[0][1]*N[1]
    N[2] -= N[2][1]*N[1]
    N[3] -= N[3][1]*N[1]

    pivot = N[2][2]
    N[2] /= pivot
    N[0] -= N[0][2]*N[2]
    N[1] -= N[1][2]*N[2]
    N[3] -= N[3][2]*N[2]

    v = np.array([N[j][3] for j in range(3)])
    return vector.normalize(np.array([*v, -1]))
