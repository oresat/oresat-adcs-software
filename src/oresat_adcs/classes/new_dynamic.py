import numpy as np
# For type hints
from numpy.typing import NDArray

from ..functions import frame, quaternion, vector
from ..configuration import structure, environment
from . import jday, sensor



class ReducedDynamicalSystem():
    '''This class stores the state vector, Julian date, reference frame transformations,
    satellite structural model, sensor models, and environmental models. It also calculates the vector field for the states.
    This is a reduced version of dynamical system which only cares about position and velocity.

    Parameters
    ----------
    position : numpy.ndarray : Initial position in ECI coordinates
    lin_vel : numpy.ndarray : Initial velocity in ECI coordinates
    date_and_time : list : Initial date and time. (Y, M, D, h, m, s) format.
    satellite : oresat_adcs.configruation.structure.ReducedSatellite
        Satellite instance where the products of moment of inertia are NOT implemented (reduced=False) (Does this really even matter?)
        satellite will be used for observer
        This satellite may be a reduced satellite (I think)
    '''
    def __init__(self, position, lin_vel, date_and_time, satellite, reduced_environment):
        self.simulator   = False
        self.state       = np.array([position, lin_vel])

        self.satellite = satellite

        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)
        self.update_transformation_matrices()
        self.enviro      = reduced_environment

    def update_transformation_matrices(self):
        '''Some of the reference frame transformations depend on the current state vector.
        At the moment, the only other reference frame we care about is ECEF, which the GPS uses.
        This method just updates the transformations to the current state.
        '''
        self.GCI_to_ECEF = frame.inertial_to_ecef(self.clock)

   def relative_vel(self, x: NDArray, v: NDArray) -> NDArray:
        '''Relative velocity of satellite, with respect to the atmosphere at this location,
        assuming atmosphere rotates with earth with respect to inertial frame.

        Parameters
        ----------
        x : Position of satellite in inertial frame.
        v : Velocity of satellite in inertial frame.

        Returns
        -------
        Relative velocity of satellite.
        '''
        v_rel = np.array([v[0] + self.EARTH_ROTATION * x[1],
                          v[1] + self.EARTH_ROTATION * x[0],
                          v[2]])
        return v_rel # m/s


   def solar_radiation_force(self, clock, x):
        '''Unit vector in inertial coordinates pointing from satellite to the sun.
        More details on page 420 of Markely & Crassidis. For now assume the sun is a constant distance away.

        Parameters
        ----------
        clock : jday.Clock : Clock is needed because the position of the sun depends on the date and time (obviously).
        x : numpy.ndarray : Position of the satellite in inertial frame.

        Returns
        -------
        numpy.ndarray : Inertial unit vector pointing at sun from satellite.
        '''
        # pass the solar radiation pressure parameters to satellite
        return (- self.satellite.solar_radiation_force(self.enviro.solar_radiation_pressure(clock, x)))



    def drag_force(self, v):
        '''Calculates drag force
        
        Parameters:
        -----------
        rho : float : the density of air 
        v : numpy.ndarray : Position of the satellite in inertial frame.
        '''
        return self.satellite.drag_force(rho, v)


    def gravity_force(self, x):
        '''Gravitational acceleration (m/s^2) of satellite.

        Parameters
        ----------
        x : numpy.ndarray
            Position of satellite in inertial frame.
        '''
        length = np.linalg.norm(x)
        coeff  = self.MU / length**3
        accel  = -coeff * position
        return accel * self.satellite.mass


    def forces(self, x, v):
        '''Forces acting on satellite.

        Parameters
        ----------
        x : numpy.ndarray : Position of satellite in inertial frame. (ECI?)
        v : numpy.ndarray : Velocity of satellite in inertial frame. (ECI?)

        Returns
        -------
        numpy.ndarray : Force acting on the satellite
        '''
        v_rel = self.relative_vel(x, v)
        D = self.drag_force(v_rel)
        G = self.gravity_force(x)
        return D + G


    def vector_field(self, position, lin_vel):
        '''This is the vector field for our state space. It defines the differential equation to be solved.

        Parameters
        ----------
        position : numpy.ndarray : Present position in ECI coordinates
        lin_vel : numpy.ndarray : Present velocity in ECI coordinates

        Returns
        -------
        numpy.ndarray : Array of arrays for derivatives of state variables.
        '''
        F = self.forces(position, lin_vel)
        dxdt, dvdt = lin_vel, F / self.satellite.mass
        return np.array([dxdt, dvdt])




class DynamicalSystem(ReducedDynamicalSystem):
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
    def __init__(self, position, lin_vel, attitude, body_ang_vel, wheel_vel, date_and_time, satellite, environment):
        self.simulator   = True
        self.state       = np.array([position, lin_vel, attitude, body_ang_vel, wheel_vel], dtype=object)
        self.init_date   = date_and_time
        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)
        self.update_transformation_matrices()
        
        # Define the satellite, it should check for the correct object type
        self.satellite = satellite

        self.enviro      = environment
        #making up std dev as placeholders
        self.sensors     = [sensor.GPS_pos(mean=0, std_dev=30, model=self),
                            sensor.GPS_vel(mean=0, std_dev=2, model=self),
                            sensor.StarTracker(mean=0, std_dev=0.75e-7, model=self),
                            sensor.Gyro(arw_mean=0, arw_std_dev=2.79e-4, rrw_mean=0, rrw_std_dev=8.73e-7, init_bias=3.15e-5, model=self),
                            sensor.Wheel_vel(mean=0, std_dev=0.0001, model=self),
                            sensor.Magnetometer(mean=0, std_dev=4e-8, model=self), # from datasheet
                            sensor.SunSensor(mean=0, std_dev=1e-6, model=self)
                            ]


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


    def srp_torque(self, srp_force):
        pass


    def drag_torque(self, attitude, v, v_norm, pressure):
        '''Find the drag torque on the satellite'''
        v_ref    = quaternion.sandwich(attitude, v / v_norm)
        # this is the algorithm used in the old satellite structure
        drag_torque = sum([wall.drag_torque() for wall in self.satellite.walls])
        return F_and_T


    def gravity_torque(self, position, attitude, MoI):
        '''Find the gravity torque'''
        length = np.linalg.norm(position)
        coeff  = self.MU / length**2
        n      = quaternion.sandwich(attitude, -position / length)
        T      = 3 * coeff / length * np.cross(n, MoI.dot(n))
        return T

    def forces(self, x, v, q):
        '''Gets the forces acting on the satellite and transforms them to a coordinate system'''
        blah_forces = super().forces(x, v)
        # apply coordinate transformation
        return quaternion.sandwich_opp(q, blah_forces)


    def torques():
        '''Gets the torques acting on the satellite'''
        r_ecef       = self.GCI_to_ECEF.dot(position)
        length       = np.linalg.norm(position)

        srp_torque = self.srp_torque()
        drag_torque = self.drag_torque(q, v, v_norm, pressure)
        gravity_torque = self.gravity_torque(position, attitude, MoI)

        mag_field            = self.magnetic_field(r_ecef, length, GCI_to_ECEF)
        mag_field_body       = quaternion.sandwich(attitude, B)
        mag_torque = np.cross(mag_moment, mag_field_body)
        pass



    def vector_field(self, position, lin_vel, attitude, body_ang_vel, wheel_vel,
                     cur_cmd, whl_accl):
        '''This is the vector field for our state space. It defines the differential equation to be solved.

        Parameters
        ----------
        position : numpy.ndarray : Present position in ECI coordinates
        lin_vel : numpy.ndarray : Present velocity in ECI coordinates
        attitude : numpy.ndarray : Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray : Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray : Present velocities of reaction wheels.
        body_ang_accl : numpy.ndarray : Present (i.e. last) angular acceleration
        cur_cmd : numpy.ndarray : Magnetorquer commands for currents.
        whl_accl : numpy.ndarray : Reaction wheel acceleration commands.

        Returns
        -------
        numpy.ndarray : Array of arrays for derivatives of state variables.
        '''

        #attitude     = vector.normalize(attitude) # we could do this for the intermediate RK4 steps, probably not necessary
        mag_moment   = self.satellite.magnetorquers.actuate(cur_cmd)
        
        F_env = self.forces(position, lin_vel, attitude, self.clock, mag_moment)
        T_env = self.torques()

        F_env, T_env = self.enviro.env_F_and_T(position, lin_vel, attitude, self.clock, mag_moment)
        T_whl        = self.satellite.reaction_wheels.torque(whl_accl)
        H_whl        = self.satellite.reaction_wheels.momentum(wheel_vel)

        dxdt, dvdt   = (lin_vel, F_env / self.satellite.mass)
        dqdt         = 0.5 * quaternion.product(attitude, np.array([0, body_ang_vel[0], body_ang_vel[1], body_ang_vel[2]]))
        dwdt         = self.satellite.inv_tot_moment.dot(T_env - T_whl - np.cross(body_ang_vel, H_whl + self.satellite.total_moment.dot(body_ang_vel)))
        dw_rw_dt     = whl_accl
        return np.array([dxdt, dvdt, dqdt, dwdt, dw_rw_dt], dtype=object)









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
