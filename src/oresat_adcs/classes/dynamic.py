import numpy as np
from ..functions import frame, quaternion, vector
from ..configuration import structure, environment
from . import jday, sensor

class DynamicState():
    '''This class represents the state vector values, rather than having a list of numpy values'''

    def __init__(self, position, lin_vel, attitude, body_ang_vel, wheel_vel, clock):
        '''
        Parameters
        ----------
        position : numpy.ndarray
            Initial position in ECI coordinates
        lin_vel : numpy.ndarray
            Initial velocity in ECI coordinates
        attitude : numpy.ndarray
            Initial attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Initial angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Initial velocities of reaction wheels
        clock : jday.JClock
            Clock of the initial date and time
        '''
        self.position = position
        self.lin_vel = lin_vel
        self.attitude = attitude
        self.body_ang_vel = body_ang_vel
        self.wheel_vel = wheel_vel
        self.clock = clock


class DynamicalSystem():
    '''This class stores the state vector, Julian date, reference frame transformations,
    satellite structural model, sensor models, and environmental models. It also calculates the vector field for the states.

    Parameters
    ----------
    position : numpy.ndarray
        Initial position in ECI coordinates
    lin_vel : numpy.ndarray
        Initial velocity in ECI coordinates
    attitude : numpy.ndarray
        Initial attitude quaternion with respect to inertial frame.
    body_ang_vel : numpy.ndarray
        Initial angular velocity in body frame coordinates with respect to inertial frame.
    wheel_vel : numpy.ndarray
        Initial velocities of reaction wheels.
    date_and_time : list
        Initial date and time. (Y, M, D, h, m, s) format.
    satellite : oresat_adcs.configruation.strucutre.Satellite
        Satellite instance where the products of moment of inertia are NOT implemented (reduced=False) (Does this really even matter?)
    '''
    def __init__(self, position, lin_vel, attitude, body_ang_vel, wheel_vel, date_and_time, satellite):
        self.simulator   = True
        self.state       = np.array([position, lin_vel, attitude, body_ang_vel, wheel_vel], dtype=object)
        self.init_date   = date_and_time
        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)
        self.update_transformation_matrices()
        
        # Define the satellite, it should check for the correct object type
        self.satellite = satellite

        self.enviro      = environment.Environment(self.satellite, hi_fi=True)
        #making up std dev as placeholders
        self.sensors     = [sensor.GPS_pos(mean=0, std_dev=30, model=self),
                            sensor.GPS_vel(mean=0, std_dev=2, model=self),
                            sensor.StarTracker(mean=0, std_dev=0.75e-7, model=self),
                            sensor.Gyro(arw_mean=0, arw_std_dev=2.79e-4, rrw_mean=0, rrw_std_dev=8.73e-7, init_bias=3.15e-5, model=self),
                            sensor.Wheel_vel(mean=0, std_dev=0.0001, model=self),
                            sensor.Magnetometer(mean=0, std_dev=4e-8, model=self), # from datasheet
                            sensor.SunSensor(mean=0, std_dev=1e-6, model=self)
                            ]

    def update_transformation_matrices(self):
        '''Some of the reference frame transformations depend on the current state vector.
        At the moment, the only other reference frame we care about is ECEF, which the GPS uses.
        This method just updates the transformations to the current state.
        '''
        self.GCI_to_ECEF = frame.inertial_to_ecef(self.clock)

    def vector_field(self, position, lin_vel, attitude, body_ang_vel, wheel_vel,
                     cur_cmd, whl_accl):
        '''This is the vector field for our state space. It defines the differential equation to be solved.

        Parameters
        ----------
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        body_ang_accl : numpy.ndarray
            Present (i.e. last) angular acceleration
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.

        Returns
        -------
        numpy.ndarray
            Array of arrays for derivatives of state variables.
        '''
        #attitude     = vector.normalize(attitude) # we could do this for the intermediate RK4 steps, probably not necessary
        mag_moment   = self.satellite.magnetorquers.actuate(cur_cmd)
        F_env, T_env = self.enviro.env_F_and_T(position, lin_vel, attitude, self.clock, self.GCI_to_ECEF, mag_moment)
        T_whl        = self.satellite.reaction_wheels.torque(whl_accl)
        H_whl        = self.satellite.reaction_wheels.momentum(wheel_vel)

        dxdt, dvdt   = (lin_vel, F_env / self.satellite.mass)
        dqdt         = 0.5 * quaternion.product(attitude, np.array([0, body_ang_vel[0], body_ang_vel[1], body_ang_vel[2]]))
        dwdt         = self.satellite.inv_tot_moment.dot(T_env - T_whl - np.cross(body_ang_vel, H_whl + self.satellite.total_moment.dot(body_ang_vel)))
        dw_rw_dt     = whl_accl
        return np.array([dxdt, dvdt, dqdt, dwdt, dw_rw_dt], dtype=object)

    def measurement(self, noisy):
        '''Measures the present state of the system. In order:
        position, velocity, attitude, angular rate, wheel velocities, magnetic field, sun vector.

        Parameters
        ----------
        noisy : bool
            True if measurements should contain noise, False if the true state should be returned.

        Returns
        -------
        list
            Contains numpy.ndarrays for measurements.
        '''
        return [sens.measurement(noisy) for sens in self.sensors]

class ReducedDynamicalSystem(DynamicalSystem):
    '''This class stores the state vector, Julian date, reference frame transformations,
    satellite structural model, sensor models, and environmental models. It also calculates the vector field for the states.
    This is a reduced version of dynamical system which only cares about position and velocity.

    Parameters
    ----------
    position : numpy.ndarray
        Initial position in ECI coordinates
    lin_vel : numpy.ndarray
        Initial velocity in ECI coordinates
    date_and_time : list
        Initial date and time. (Y, M, D, h, m, s) format.
    satellite : oresat_adcs.configruation.structure.ReducedSatellite
        Satellite instance where the products of moment of inertia are NOT implemented (reduced=False) (Does this really even matter?)
        satellite will be used for observer
        This satellite may be a reduced satellite (I think)
    '''
    def __init__(self, position, lin_vel, date_and_time, satellite):
        self.simulator   = False
        self.state       = np.array([position, lin_vel])

        self.satellite = satellite

        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)
        self.update_transformation_matrices()
        self.enviro      = environment.ReducedEnvironment()

    def vector_field(self, position, lin_vel):
        '''This is the vector field for our state space. It defines the differential equation to be solved.

        Parameters
        ----------
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates

        Returns
        -------
        numpy.ndarray
            Array of arrays for derivatives of state variables.
        '''
        F = self.enviro.forces(position, lin_vel)
        dxdt, dvdt = lin_vel, F / self.satellite.mass
        return np.array([dxdt, dvdt])

class Integrator():
    '''This is a numerical integrator for propagating an initial state through state space.

    Parameters
    ----------
    model : DynamicalSystem
        Model that is to be numerically integrated.
    dt : float
        Fixed time-step.
    '''
    def __init__(self, model, dt):
        self.model         = model
        self.f             = self.model.vector_field
        self.dt            = dt

    def RK4_step(self, state, param):
        '''This is the 4th order Runge-Kutta method. We could exchange this method for any other algorithm.
        However, this is a nice general purpose tool. Error is on the order of dt^5.

        Parameters
        ----------
        state : numpy.ndarray
            Last state of system.
        param : list
            Any exogenous inputs to system.

        Returns
        -------
        numpy.ndarray
            State of system at next time step.
        numpy.ndarray
            The angular acceleration, in the case of a DynamicalSystem model.
        '''
        k1     = self.f(*state, *param)
        k2     = self.f(*(state + k1 * self.dt/2), *param)
        k3     = self.f(*(state + k2 * self.dt/2), *param)
        k4     = self.f(*(state + k3 * self.dt), *param)
        change = (k1 + 2*k2 + 2*k3 + k4) / 6
        return state + change * self.dt

    def update(self, state, param):
        '''This takes the model one step forward and updates its clock, transformations, and sensors.

        Parameters
        ----------
        state : numpy.ndarray
            Last state of system.
        param : list
            Any exogenous inputs to system.
        '''
        next_step         = self.RK4_step(state, param)
        if self.model.simulator:
            next_step[2]  = vector.normalize(next_step[2])
        self.model.state  = next_step
        self.model.clock.tick(self.dt)
        self.model.update_transformation_matrices()
        if self.model.simulator:
            self.model.sensors[3].propagate(self.dt)

    def integrate(self, duration, zero_order_hold):
        '''This is a front-facing interface for the library. It takes an integration duration and a set of fixed exogenous commands.
        Then it propagates the dynamic model for that duration using those commands.

        Parameters
        ----------
        duration : float
            The length of time (s) to integrate over.
        zero_order_hold : list
            Two numpy.ndarrays, for current commands and acceleration commands.
        '''
        t = self.dt
        while t < duration or abs(t - duration) < 0.001:
            if self.model.simulator:
                cur_cmd  = zero_order_hold[0]
                accl_cmd = zero_order_hold[1]
                self.update(self.model.state, [cur_cmd, accl_cmd])
            else:
                self.update(self.model.state, [])
            t += self.dt
