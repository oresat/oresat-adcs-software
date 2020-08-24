import numpy as np
from adcs_lib import frame, structure, environment, jday, quaternion, sensor

class DynamicalSystem():
    '''This class stores the state vector, Julian date, reference frame transformations,
    satellite structural model, and environmental models. It also calculates the vector field for the states.'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self, position, lin_vel, attitude, body_ang_vel, wheel_vel, date_and_time):
        self.state       = np.array([position, lin_vel, attitude, body_ang_vel, wheel_vel], dtype=object)
        self.init_date   = date_and_time
        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)
        self.update_transformation_matrices()
        self.satellite   = structure.Satellite(structure.LENGTH, structure.WIDTH, structure.HEIGHT, structure.PRINCIPAL,
                                               structure.INCLINATION, structure.AZIMUTH, structure.PARALLEL, structure.ORTHOGONAL,
                                               0.0005, False)
        self.enviro      = environment.Environment(self.satellite, hi_fi=True)
        self.simulator   = True
        #making up std dev as placeholders
        self.sensors     = [sensor.GPS_pos(mean=0, std_dev=30, model=self),
                            sensor.GPS_vel(mean=0, std_dev=2, model=self),
                            sensor.StarTracker(mean=0, std_dev=1.5e-7/2, model=self),
                            sensor.Gyro(arw_mean=0, arw_std_dev=2.79e-4, rrw_mean=0, rrw_std_dev=8.73e-4, init_bias=3.15e-5, model=self),
                            sensor.Wheel_vel(mean=0, std_dev=0.0174533, model=self),
                            sensor.Magnetometer(mean=0, std_dev=1e-6, model=self),
                            sensor.SunSensor(mean=0, std_dev=1e-6, model=self)
                            ]

    def update_transformation_matrices(self):
        '''Some of the reference frame transformations depend on the current state vector.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        self.GCI_to_ECEF = frame.inertial_to_ecef(self.clock)

    def vector_field(self, position, lin_vel, attitude, body_ang_vel, wheel_vel,
                     body_ang_accl, cur_cmd, whl_accl):
        '''This is the vector field for our state space. It defines the differential equation to be solved.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        attitude     = quaternion.normalize(attitude) # we do this for the intermediate RK4 steps
        mag_moment   = self.satellite.magnetorquers.actuate(cur_cmd)
        F_env, T_env = self.enviro.env_F_and_T(position, lin_vel, attitude, self.GCI_to_ECEF, mag_moment)
        T_whl        = self.satellite.reaction_wheels.torque(whl_accl, body_ang_accl)
        H_whl        = self.satellite.reaction_wheels.momentum(wheel_vel, body_ang_vel)

        dxdt, dvdt   = (lin_vel, F_env / self.satellite.mass)
        dqdt         = 0.5 * quaternion.product(attitude, np.array([0, body_ang_vel[0], body_ang_vel[1], body_ang_vel[2]]))
        dwdt         = self.satellite.inv_red_moment.dot(T_env - T_whl - np.cross(body_ang_vel, H_whl + self.satellite.reduced_moment.dot(body_ang_vel)))
        dw_rw_dt     = whl_accl
        return np.array([dxdt, dvdt, dqdt, dwdt, dw_rw_dt], dtype=object)

    def measurement(self, noisy):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return [sens.measurement(noisy) for sens in self.sensors]

class ReducedDynamicalSystem(DynamicalSystem):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self, position, lin_vel, date_and_time):
        self.state       = np.array([position, lin_vel])
        self.satellite   = structure.Satellite(structure.LENGTH, structure.WIDTH, structure.HEIGHT, structure.PRINCIPAL,
                                               structure.INCLINATION, structure.AZIMUTH, structure.PARALLEL, structure.ORTHOGONAL,
                                               0.0005, False)
        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)
        self.update_transformation_matrices()
        self.enviro      = environment.ReducedEnvironment()
        self.simulator   = False

    def vector_field(self, position, lin_vel):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        F = self.enviro.forces(position, lin_vel)
        dxdt, dvdt = lin_vel, F / self.enviro.M
        return np.array([dxdt, dvdt])

class Integrator():
    '''This is a numerical integrator for propagating an initial state through state space.'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self, model, dt):
        self.model         = model
        self.f             = self.model.vector_field
        self.dt            = dt
        self.last_ang_accl = np.zeros(3)

    def RK4_step(self, state, param):
        '''This is the 4th order Runge-Kutta method. We could exchange this method for any other algorithm.
        However, this is a nice general purpose tool. Error is on the order of dt^5.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        k1     = self.f(*state, *param)
        k2     = self.f(*(state + k1 * self.dt/2), *param)
        k3     = self.f(*(state + k2 * self.dt/2), *param)
        k4     = self.f(*(state + k3 * self.dt), *param)
        change = (k1 + 2*k2 + 2*k3 + k4) / 6
        return state + change * self.dt, change[-2]

    def update(self, state, param):
        '''This takes the model one step forward and updates its clock, transformations, and sensors.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        next_step, self.last_ang_accl = self.RK4_step(state, param)
        if self.model.simulator:
            next_step[2]                  = quaternion.normalize(next_step[2])
        self.model.state              = next_step
        self.model.clock.tick(self.dt)
        self.model.update_transformation_matrices()
        if self.model.simulator:
            self.model.sensors[3].propagate(self.dt)

    def integrate(self, duration, zero_order_hold):
        '''This is the front-facing interface for the library. It takes an integration duration and a set of fixed exogenous commands.
        Then it propagates the dynamic model for that duration using those commands.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        t = self.dt
        while t < duration or abs(t - duration) < 0.001:
            if self.model.simulator:
                cur_cmd = zero_order_hold[0]
                accl_cmd   = zero_order_hold[1]
                self.update(self.model.state, [self.last_ang_accl, cur_cmd, accl_cmd])
            else:
                self.update(self.model.state, [])
            t += self.dt
