import numpy as np
import json
from ..functions import frame, quaternion, vector
from ..classes import jday, sensor
from . import structure, environment


class OrbitalState():
    '''State for orbital mechanics'''
    pass

class AttitudeState():
    '''State for attitude'''
    pass


class ReducedSatellite():
    '''Simple satellite model for reduced models'''

    def __init__(self, environment, mass, drag_coefficient, area, position, lin_vel, date_and_time):
        self.mass = mass
        self.drag_coefficient = drag_coefficient
        self.area = area
        self.absorption = absorption
        self.reflection = 1 - absorption

        self.state = np.array([position, lin_vel])
        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)
        self.enviro      = reduced_environment

        # replace this with frame directly
        self.update_transformation_matrices()
        # relative velocity should be a part of frame too???

    def forces(self, x, v):
        '''Forces acting on satellite.

        Parameters
        ----------
        x (numpy.ndarray): Position of satellite in inertial frame.
        v (numpy.ndarray): Velocity of satellite in inertial frame.

        Returns
        -------
        numpy.ndarray: Total forces acting on satellite.
        '''
        
        v_rel = self.enviro.relative_vel(x, v)
        length = np.linalg.norm(position)
        v_norm   = np.linalg.norm(v_rel)
        # drag force
        D = = -0.5 * self.enviro.atmo_density() * self.drag_coefficient * self.area * v_norm * v
        # gravity force
        G = -(self.MU/length**3) * x * self.mass
        # alternatively use something like this
        # G = self.enviro.gravity_accel() * x * self.mass
        
        # also have SRP force, assume srp is a pressure vector
        # S = self.enviro.srp() * (2*self.reflection + self.absorption) * self.area
        return D + G # + S


    def vector_field(self, position, lin_vel):
        '''This is the vector field for our state space. 
        It defines the differential equation to be solved.

        Parameters
        ----------
        position (numpy.ndarray) : Present position in ECI coordinates
        lin_vel (numpy.ndarray) : Present velocity in ECI coordinates

        Returns
        -------
        numpy.ndarray: Array of arrays for derivatives of state variables.
        '''
        F = self.forces(position, lin_vel)
        dxdt, dvdt = lin_vel, F / self.mass
        return np.array([dxdt, dvdt])







def load(config_file_path, max_T, torque_limited=True):
        '''Class function to create a satellite object

        Parameters
        ----------
        config_file_path : str
            Path to json configuration file, see examples
        max_T : float
            Config parameter to limite the maximum torque of the reaction wheel system
        torque_limited : bool
            Config parameter to limit the reaction wheels

        Returns
        -------
        oresat_adcs.configuration.structure.Satellite object
            Satellite object built from config file
        '''
        
        def apply_nparray(keys, dictionary):
            """Helper function to make certain lists into numpy arrays

            Parameters
            ----------
            keys : list
                List of keys whos values should be changed to numpy arrays
            dictionary : dict
                Configuration dictionary to match the keyword cards of classes

            Returns
            -------
            dictionary : dict
                Dictionary that values properly matching keyword arguments
            """
            for key in keys:
                dictionary[key] = np.array(dictionary[key])
            return dictionary


        def make_objects_list(target_class, config_dict):
            """Helper function to return a list of objects based on a confituration dictionary"""
            object_list = []
            for name,config in config_dict.items():
                nparray_config = apply_nparray(config["nparrays"], config["keywords"])
                object_list.append(target_class(**nparray_config))

            return object_list


        with open(config_file_path, 'r') as structure_config:
            config_dict = json.load(structure_config)

        instrument_list = make_objects_list(SensitiveInstrument, config_dict["instruments"])
        magnetorquer_list = make_objects_list(Magnetorquer, config_dict["magnetorquers"])
        reaction_wheel_list = make_objects_list(Wheel, config_dict["reaction_wheels"])

        mt_system = MagnetorquerSystem(magnetorquer_list)
        rw_system = ReactionWheelSystem(reaction_wheel_list, max_T, torque_limited)

        sat_config_kwargs = apply_nparray(config_dict["satellite"]["nparrays"], config_dict["satellite"]["keywords"])

        return Satellite(**sat_config_kwargs, rw_sys=rw_system, mt_sys=mt_system, sensitive_instruments=instrument_list)









class Satellite():
    '''A rectangular prism satellite and its relevant material properties.
    At some point, add support for products of inertia, and parametrize magnetorquers.

    Note that if you want a reduced dynamical system or reduced environment, products of inertia should be set to False

    Parameters
    ----------
    dimensions : np.array
        Dimension of satellite walls in meters: [length, width, height]
    principal_moments : numpy.ndarray
        Principal moments of inertia for satellite, minus reaction wheels.
    products_of_inertia : bool
        Changes the moment of inertia, set to False if you use a reduced dynamical system or reduced environement
    reaction_wheel_system : oresat_adcs.configuration.structure.ReactionWheelSystem
        A custom ReactionWheelSystem instance to use
    magnetorquer_system : oresat_adcs.configuration.structure.MagnetorquerSystem
        A custom ReactionWheelSystem instance to use
    sensitive_instruments : list(oresat_adcs.configuraiton.structure.SensitiveInstrument)
        A list of sensitive instrument instances
    '''
    def __init__(self, mass, dimensions, absorption, drag_coeff, principal_moments, product_moments, reduced, sensors, rw_sys, mt_sys, sensitive_instruments=[]):
        
        # The satellite should define its own dimensions and walls
        self.length         = dimensions[0]
        self.width          = dimensions[1]
        self.height         = dimensions[2]
        self.walls          = (Wall(self.length/2, np.array([1, 0, 0]), self.width, self.height, absorption),
                               Wall(self.length/2, np.array([-1, 0, 0]), self.width, self.height, absorption),
                               Wall(self.width/2, np.array([0, 1, 0]), self.length, self.height, absorption),
                               Wall(self.width/2, np.array([0, -1, 0]), self.length, self.height, absorption),
                               Wall(self.height/2, np.array([0, 0, 1]), self.width, self.length, absorption),
                               Wall(self.height/2, np.array([0, 0, -1]), self.width, self.length, absorption))

        
        #: Estimated drag coefficient.
        self.drag_coeff      = drag_coeff
        self.mass            = mass

        self.sensors     = sensors
        self.reaction_wheels = rw_sys
        self.magnetorquers = mt_sys
        self.instruments = sensitive_instruments
        
        #: Moment of inertia for reaction wheels about spin axes with respect to principal axes.
        self.wheel_moment    = self.reaction_wheels.parallel_moment

        #: Moment of inertia for the satellite except the moments of wheels about spin axes.
        self.reduced_moment  = np.diag(principal_moments) + self.reaction_wheels.orthogonal_moment
        # If the model is reduced, the following can be skipped
        if reduced:
            self.reduced_moment += np.array([[0 if i == j else product_moments[i + j - 1] for i in range(3)] for j in range(3)])

        #: Total moment of inertia of the satellite.
        self.total_moment    = self.reduced_moment + self.wheel_moment
        #: Inverse of the moment of inertia for the satellite except the moments of wheels about spin axes.
        self.inv_red_moment  = np.linalg.inv(self.reduced_moment)
        self.inv_tot_moment  = np.linalg.inv(self.total_moment)


        # Dynamics
        self.simulator   = True
        self.state       = np.array([position, lin_vel, attitude, body_ang_vel, wheel_vel], dtype=object)
        self.init_date   = date_and_time
        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)


        # just call frame or make it part of the environment
        self.update_transformation_matrices()

    
    def measurement(self, noisy):
        '''Measures the present state of the system. In order:
        position, velocity, attitude, angular rate, wheel velocities, magnetic field, sun vector.

        Parameters
        ----------
        noisy (bool): True if measurements should contain noise, False if the true state should be returned.

        Returns
        -------
        list: Contains numpy.ndarrays for measurements.
        '''
        return [sens.measurement(noisy) for sens in self.sensors]




    def area_and_cop(self, v_ref):
        '''Projected surface area and center of pressure for whole satellite.
        Note that I (Cory) don't have a great deal of confidence in this CoP calculation.
        At some point, take a more rigorous look at this.

        Parameters
        ----------
        v_ref : numpy.ndarray
            Vector defining the plane the surfaces are projected onto.

        Returns
        -------
        tuple
            First entry is the projected surface area, second is the center of pressure vector.
        '''
        (A, CoP) = sum([wall.center_of_pressure(v_ref) for wall in self.walls])

        return (A, CoP / A)



    def solar_F_and_T(self, clock, x):
        '''Calculates solar radiation pressure torque on this wall.
        Assumes there is no diffuse reflection.

        Parameters
        ----------
        clock (jday.Clock): Clock is needed because the position of the sun depends on the date and time (obviously).
        x (numpy.ndarray): Position of the satellite in inertial frame.


        Returns
        -------
        numpy.ndarray
            Force (N) and torque (N m) on satellite.
        '''
        SRP, in_shadow, S_inertial = self.enviro.SRP_info(clock, x)
        
        if in_shadow:
            return np.array([np.zeros(3), np.zeros(3)])
        
        solar_F_and_T = sum([wall.srp_force(SRP, S_inertial) for wall in self.walls])
        # check for coordinate transformation
        solar_F_and_T[0] = quaternion.sandwich_opp(q, solar_F_and_T[0])
        return solar_F_and_T



    def drag_F_and_T(self, v_rel):
        
        drag_v_norm   = np.linalg.norm(v_rel)
        drag_v_ref    = quaternion.sandwich(attitude, v_rel / drag_v_norm)
        drag_pressure = -0.5 * self.enviro.atmo_density(h) * self.drag_coeff * drag_v_norm**2

        F_and_T = sum([wall.drag_force(drag_pressure, drag_v_ref) for wall in self.walls])

        # coordinate transformation
        F_and_T[0] = quaternion.sandwich_opp(attitude, F_and_T[0])
        return F_and_T


    def gravity_F_and_T(self, position, lenght, attitude):
        '''Gravitational forces and torques.
        Note that gravity torque is fairly predictable and we could even use it for feedforward in controls.

        Parameters
        ----------
        position (numpy.ndarray): Position of satellite in inertial frame.
        length (float): Norm of position vector.
        attitude (numpy.ndarray): Attitude of satellite.

        Returns
        -------
        numpy.ndarray : gravity force and torque vector
        '''
        coeff  = self.MU / length**2
        if self.hi_fi:
            accel = self.hi_fi_gravity(position, length, coeff)
        else:
            accel = -coeff * position / length
        n      = quaternion.sandwich(attitude, -position / length)
        T      = 3 * coeff / length * np.cross(n, self.total_moment.dot(n))


        # check for coordinate transformation
        return np.array([accel*self.mass, T])



    def env_F_and_T(self, position, velocity, attitude, clock, mag_moment):
        '''External forces and torques acting on satellite.

        Parameters
        ----------
        position (numpy.ndarray): Position of satellite in inertial frame.
        velocity (numpy.ndarray): Velocity of satellite in inertial frame.
        attitude (numpy.ndarray): Attitude of satellite.
        GCI_to_ECEF (numpy.ndarray): Matrix for coordinate transformation from inertial frame to ECEF frame.
        mag_moment (numpy.ndarray): Magnetic dipole moment of satellite.

        Returns
        -------
        numpy.ndarray: Array of arrays for all forces and torques acting on satellite.
        '''
        GCI_to_ECEF = frame.inertial_to_ecef(self.clock)
        r_ecef       = GCI_to_ECEF.dot(position)
        length       = np.linalg.norm(position)

        # do high fidelity gravity, alititude may change
        lat, long, h = frame.ecef_to_lla(r_ecef)
        rho          = self.atmo_density(h)
        

        # Solar
        S_blah = self.solar_F_and_T(clock, position)

        # drag
        v_rel = self.relative_vel(position, velocity)
        D = self.drag_F_and_T(v_rel)
        
        # gravity, make sure it is hi-fi or so
        G = self.gravity_F_and_T(position, length, attitude)

        # Magnetic Field
        B = self.enviro.magnetic_field(r_ecef, length, GCI_to_ECEF)
        B_body = quaternion.sandwich(attitude, B)
        M = np.array([np.zeros(3), np.cross(mag_moment, B_body)])

        # Put the forces in the correct reference frame
        # forces = quaternion.sandwich(attitude, old_forces)
        return D + G + M + srp


    def vector_field(self, position, lin_vel, attitude, body_ang_vel, wheel_vel,
                     cur_cmd, whl_accl):
        '''This is the vector field for our state space. It defines the differential equation to be solved.

        Parameters
        ----------
        position (numpy.ndarray): Present position in ECI coordinates
        lin_vel (numpy.ndarray): Present velocity in ECI coordinates
        attitude (numpy.ndarray): Present attitude quaternion with respect to inertial frame.
        body_ang_vel (numpy.ndarray): Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel (numpy.ndarray): Present velocities of reaction wheels.
        body_ang_accl (numpy.ndarray): Present (i.e. last) angular acceleration
        cur_cmd (numpy.ndarray): Magnetorquer commands for currents.
        whl_accl (numpy.ndarray): Reaction wheel acceleration commands.

        Returns
        -------
        numpy.ndarray: Array of arrays for derivatives of state variables.
        '''
        #attitude     = vector.normalize(attitude) # we could do this for the intermediate RK4 steps, probably not necessary
        mag_moment   = self.magnetorquers.actuate(cur_cmd)
        T_whl        = self.reaction_wheels.torque(whl_accl)
        H_whl        = self.reaction_wheels.momentum(wheel_vel)

        F_env, T_env = self.enviro.env_F_and_T(position, lin_vel, attitude, self.clock, mag_moment)

        dxdt, dvdt   = (lin_vel, F_env / self.mass)
        dqdt         = 0.5 * quaternion.product(attitude, np.array([0, body_ang_vel[0], body_ang_vel[1], body_ang_vel[2]]))
        dwdt         = self.satellite.inv_tot_moment.dot(T_env - T_whl - np.cross(body_ang_vel, H_whl + self.satellite.total_moment.dot(body_ang_vel)))
        dw_rw_dt     = whl_accl
        return np.array([dxdt, dvdt, dqdt, dwdt, dw_rw_dt], dtype=object)








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
        T_whl        = self.satellite.reaction_wheels.torque(whl_accl)
        H_whl        = self.satellite.reaction_wheels.momentum(wheel_vel)
        
        F_env, T_env = self.enviro.env_F_and_T(position, lin_vel, attitude, self.clock, self.GCI_to_ECEF, mag_moment)

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
        noisy (bool): True if measurements should contain noise, False if the true state should be returned.

        Returns
        -------
        list: Contains numpy.ndarrays for measurements.
        '''
        return [sens.measurement(noisy) for sens in self.sensors]























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



