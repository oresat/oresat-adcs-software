import numpy as np
import json
from ..functions import frame, quaternion, vector

# technically not needed
# from . import jday, sensors
# from ..configuration import environment, structure


class SatelliteState(np.ndarray):
    '''Adds attributes to numpy arrays representing states to
    improve readability.
    
    Note: updating a reference attribute `state.position = np.array([1, 0, 0])` 
    will NOT update the state, just the reference variable. The actual state is 
    part of the numpy array.
    
    Parameters:
        numpy.ndarray: an array of arrays with dtype=object
        clock: a jday.jclock'''

    def __new__(cls, input_array, clock=None, info=None):
        obj = np.asarray(input_array).view(cls)
        
        # add attributes
        obj.info = info
        obj.clock = clock
        obj.update()
        return obj

    def __array_finalize__(self, obj):
        # see InfoArray.__array_finalize__ for comments
        if obj is None: return
        self.info = getattr(obj, 'info', None)

    def __add__(self, other):
        temp = super().__add__(other)
        temp.attach_clock(self.clock)
        temp.update()
        return temp

    def attach_clock(self, clock):
        '''Adds a clock as an attribute'''
        self.clock = clock

    def update(self):
        '''Updates alias attributes and reference attributes.
        Clock must be attached first'''
        # alias variables
        self.position = self[0]
        self.velocity = self[1]
        self.attitude = self[2]
        self.body_ang_vel = self[3]
        self.wheel_vel = self[4]

        # more alias variables
        self.x = self.position
        self.v = self.velocity
        
        # Reference variables
        # remember to attach a clock
        self.GCI_to_ECEF  = frame.inertial_to_ecef(self.clock)
        self.r_ecef       = self.GCI_to_ECEF.dot(self.position)
        self.length       = np.linalg.norm(self.position)
        self.lat, self.long, self.h = frame.ecef_to_lla(self.r_ecef)
        pass



class DynamicState():
    '''This could also be the integrator???
    
    Parameters
        position (numpy.ndarray): position in inertial coorindates
        velocity (numpy.ndarray): linear velocity in inertial coordinates'''

    def __init__(self, position, velocity, attitude, body_ang_velocity, wheel_vel, clock):
        self.clock = clock
        self.update_vector(np.array([position, velocity, attitude, body_ang_velocity, wheel_vel], dtype=object))
        self.update_other()
        pass

    def __add__(self, other):
        '''Other should be a numpy array'''
        # if other is another DynamicState, thats weird and then add vectors
        # if other is a numpy array, add vectors
        return DynamicState(*(self.vector + other), self.clock)
    
    def update_vector(self, state_vector):
        '''Updates state via a numpy.ndarray'''
        self.vector = state_vector
        self.position = self.vector[0]
        self.velocity = self.vector[1]
        self.attitude = self.vector[2]
        self.body_ang_vel = self.vector[3]
        self.wheel_vel = self.vector[4]

        self.x = self.position
        self.v = self.velocity


    def update_clock(self, dt):
        '''Updates the clock and parameters that depend on clock.
        Alternatively, the clock can be tick-ed forward directly'''
        self.clock.tick(dt)
        

    def update_other(self):
        '''Other updates to do after both position and clock have been updated'''
        self.GCI_to_ECEF  = frame.inertial_to_ecef(self.clock)
        self.r_ecef       = self.GCI_to_ECEF.dot(self.position)
        self.length       = np.linalg.norm(self.position)
        self.lat, self.long, self.h = frame.ecef_to_lla(self.r_ecef)

    def vector():
        '''Returns state as numpy.ndarray for matrix simplicity'''
        return np.array()






class Dynamics():
    '''A rectangular prism satellite and its relevant material properties.
    At some point, add support for products of inertia, and parametrize magnetorquers.
    Note that if you want a reduced dynamical system or reduced environment, products of inertia should be set to False

    Parameters
        environement: class that contains models of the orbit environment
        dimensions : np.array : Dimension of satellite walls in meters: [length, width, height]
        principal_moments : numpy.ndarray : Principal moments of inertia for satellite, minus reaction wheels.
        products_of_inertia : bool : Changes the moment of inertia, set to False if you use a reduced dynamical system or reduced environement
        reaction_wheel_system : configuration.structure.ReactionWheelSystem : A custom ReactionWheelSystem instance to use
        magnetorquer_system : configuration.structure.MagnetorquerSystem : A custom ReactionWheelSystem instance to use
        sensitive_instruments : list(oresat_adcs.configuraiton.structure.SensitiveInstrument) : A list of sensitive instrument instances
    '''
    def __init__(self, satellite, environment):
        
        self.satellite = satellite 
        self.enviro = environment
        # Dynamics
        self.simulator   = True


    
    def measurement(self, state, noisy):
        '''Measures the present state of the system. In order:
        position, velocity, attitude, angular rate, wheel velocities, magnetic field, sun vector.

        Parameters
            noisy (bool): True if measurements should contain noise, False if the true state should be returned.

        Returns
            list: Contains numpy.ndarrays for measurements.
        '''
        return [sens.measurement(state, noisy) for sens in self.satellite.sensors]


    def area_and_cop(self, v_ref):
        '''Projected surface area and center of pressure for whole satellite.
        Note that I (Cory) don't have a great deal of confidence in this CoP calculation.
        At some point, take a more rigorous look at this.

        Parameters
            v_ref : numpy.ndarray : Vector defining the plane the surfaces are projected onto.

        Returns
            tuple : First entry is the projected surface area, second is the center of pressure vector.
        '''
        (A, CoP) = sum([wall.center_of_pressure(v_ref) for wall in self.satellite.walls])

        return (A, CoP / A)



    def solar_F_and_T(self, state):
        '''Calculates solar radiation pressure torque on this wall.
        Assumes there is no diffuse reflection.

        Parameters:
            clock (jday.Clock): Clock is needed because the position of the sun depends on the date and time (obviously).
            x (numpy.ndarray): Position of the satellite in inertial frame.
            attitude (numpy.ndarray): quaternion attitude in respect to the inertial frame.

        Returns
            numpy.ndarray: Force (N) and torque (N m) on satellite.
        '''
        # Get the solar pressure, shadow, and sun vector in inertial coordinates
        SRP, in_shadow, S_inertial = self.enviro.SRP_info(state)
        
        # If it is in a shadow, return nothing
        if in_shadow:
            return np.array([np.zeros(3), np.zeros(3)])
        
        # If not in shadow, get the SRP force from each wall
        solar_F_and_T = sum([wall.srp_force(SRP, S_inertial) for wall in self.satellite.walls])

        # check for coordinate transformation for forces
        solar_F_and_T[0] = quaternion.sandwich_opp(state.attitude, solar_F_and_T[0])
        return solar_F_and_T



    def drag_F_and_T(self, state):
        '''Equation for drag taking satellite altitude and orientation into account.

        Parameters: 
            rho (float): Atmospheric density.
            attitude (numpy.ndarray): Attitude of satellite.

        Returns:
            numpy.ndarray: Array of arrays for drag force and torque on satellite.
        '''
        v_rel = self.enviro.relative_vel(state)
        drag_v_norm   = np.linalg.norm(v_rel)
        drag_v_ref    = quaternion.sandwich(state.attitude, v_rel / drag_v_norm)
        drag_pressure = -0.5 * self.enviro.atmo_density(state) * self.satellite.drag_coeff * drag_v_norm**2

        F_and_T = sum([wall.drag_force(drag_pressure, drag_v_ref) for wall in self.satellite.walls])

        # check coordinate transformation for forces
        F_and_T[0] = quaternion.sandwich_opp(state.attitude, F_and_T[0])
        return F_and_T


    def gravity_F_and_T(self, state):
        '''Gravitational forces and torques.
        Note that gravity torque is fairly predictable and we could even use it for feedforward in controls.

        Parameters
            position (numpy.ndarray): Position of satellite in inertial frame.
            length (float): Norm of position vector.
            attitude (numpy.ndarray): Attitude of satellite.

        Returns
            numpy.ndarray : gravity force and torque vector
        '''
        accel = self.enviro.gravity_accel(state)
        n      = quaternion.sandwich(state.attitude, -state.position / state.length)
        T      = 3 * (self.enviro.MU) / (state.length**3) * np.cross(n, self.satellite.total_moment.dot(n))


        # check for coordinate transformation
        return np.array([accel*self.satellite.mass, T])



    def env_F_and_T(self, state, mag_moment):
        '''External forces and torques acting on satellite.

        Parameters
            position (numpy.ndarray): Position of satellite in inertial frame.
            velocity (numpy.ndarray): Velocity of satellite in inertial frame.
            attitude (numpy.ndarray): Attitude of satellite.
            clock (oresat_adcs.classes.jday.Clock): Clock object
            mag_moment (numpy.ndarray): Magnetic dipole moment of satellite.

        Returns
            numpy.ndarray: Array of arrays for all forces and torques acting on satellite.
        '''
        #orbit_now = OrbitalState(position, velocity, clock)
        #GCI_to_ECEF  = frame.inertial_to_ecef(clock)
        #r_ecef       = GCI_to_ECEF.dot(position)
        #length       = np.linalg.norm(position)
        # do high fidelity gravity, alititude may change
        #lat, long, h = frame.ecef_to_lla(r_ecef)

        # Solar forces and torques
        S_blah = self.solar_F_and_T(state)

        # drag forces and torques
        #v_rel = self.enviro.relative_vel(orbit_now)
        D = self.drag_F_and_T(state)
        
        # gravity, make sure it is hi-fi or so
        G = self.gravity_F_and_T(state)

        # Magnetic Field
        B = self.enviro.magnetic_field(state)
        B_body = quaternion.sandwich(state.attitude, B)
        M = np.array([np.zeros(3), np.cross(mag_moment, B_body)])

        # Put the forces in the correct reference frame
        # forces = quaternion.sandwich(attitude, old_forces)
        return D + G + M + S_blah


    def vector_field(self, state, cur_cmd, whl_accl):
        '''This is the vector field for our state space. It defines the differential equation to be solved.
        This should really be called state derivatives

        Parameters
            position (numpy.ndarray): Present position in ECI coordinates
            lin_vel (numpy.ndarray): Present velocity in ECI coordinates
            attitude (numpy.ndarray): Present attitude quaternion with respect to inertial frame.
            body_ang_vel (numpy.ndarray): Present angular velocity in body frame coordinates with respect to inertial frame.
            wheel_vel (numpy.ndarray): Present velocities of reaction wheels.
            body_ang_accl (numpy.ndarray): Present (i.e. last) angular acceleration
            cur_cmd (numpy.ndarray): Magnetorquer commands for currents.
            whl_accl (numpy.ndarray): Reaction wheel acceleration commands.

        Returns
            numpy.ndarray: Array of arrays for derivatives of state variables.
        '''
        #attitude     = vector.normalize(attitude) # we could do this for the intermediate RK4 steps, probably not necessary

        mag_moment   = self.satellite.magnetorquers.actuate(cur_cmd)
        T_whl        = self.satellite.reaction_wheels.torque(whl_accl)
        H_whl        = self.satellite.reaction_wheels.momentum(state.wheel_vel)

        F_env, T_env = self.env_F_and_T(state, mag_moment)

        dxdt, dvdt   = (state.velocity, F_env / self.satellite.mass)
        dqdt         = 0.5 * quaternion.product(state.attitude, np.array([0, state.body_ang_vel[0], state.body_ang_vel[1], state.body_ang_vel[2]]))
        dwdt         = self.satellite.inv_tot_moment.dot(T_env - T_whl - np.cross(state.body_ang_vel, H_whl + self.satellite.total_moment.dot(state.body_ang_vel)))
        dw_rw_dt     = whl_accl

        return np.array([dxdt, dvdt, dqdt, dwdt, dw_rw_dt], dtype=object)









class Integrator():
    '''This is a numerical integrator for propagating an initial state through state space.

    Parameters
        model : DynamicalSystem : Model that is to be numerically integrated.
        dt : float : Fixed time-step.
    '''
    def __init__(self, model, initial_state, dt):
        self.model         = model
        self.f             = self.model.vector_field
        self.state = initial_state
        self.dt            = dt

    def RK4_step(self, state, param):
        '''This is the 4th order Runge-Kutta method. We could exchange this method for any other algorithm.
        However, this is a nice general purpose tool. Error is on the order of dt^5.

        Parameters
            state (numpy.ndarray): Last state of system.
            param (list): Any exogenous inputs to system.

        Returns
            numpy.ndarray: State of system at next time step.
            numpy.ndarray: The angular acceleration, in the case of a DynamicalSystem model.
        '''
        # the clock hadn't been updated for these time steps
        # state should be updated via adding
        
        k1     = self.f(state, *param)
        k2     = self.f((state + k1 * self.dt/2), *param)
        k3     = self.f((state + k2 * self.dt/2), *param)
        k4     = self.f((state+ k3 * self.dt), *param)
        change = (k1 + 2*k2 + 2*k3 + k4) / 6
        return state + change * self.dt

    def update(self, param):
        '''This takes the model one step forward and updates its clock, transformations, and sensors.

        Parameters
            state : numpy.ndarray : Last state of system.
            param : list : Any exogenous inputs to system.
        '''
        next_step         = self.RK4_step(self.state, param)
        if self.model.simulator:
            next_step[2]  = vector.normalize(next_step[2])
        
        # Make sure transform matrix is updated correctly
        #self.model.state  = next_step
        self.state = next_step
        self.state.clock.tick(self.dt)
        self.state.update()
        # commented out for testing
        if self.model.simulator:
            self.model.satellite.sensors[3].propagate(self.dt)

    def integrate(self, duration, zero_order_hold):
        '''This is a front-facing interface for the library. It takes an integration duration and a set of fixed exogenous commands.
        Then it propagates the dynamic model for that duration using those commands.

        Parameters
            duration : float : The length of time (s) to integrate over.
            zero_order_hold : list : Two numpy.ndarrays, for current commands and acceleration commands.
        '''
        t = self.dt
        while t < duration or abs(t - duration) < 0.001:
            if self.model.simulator:
                cur_cmd  = zero_order_hold[0]
                accl_cmd = zero_order_hold[1]
                self.update([cur_cmd, accl_cmd])
            else:
                self.update([])
            t += self.dt



