import numpy as np
from ..functions import frame, quaternion, vector
from ..configuration import structure, environment
from . import jday, sensor


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
    def __init__(self, position, lin_vel, date_and_time, satellite, reduced_environment):
        self.simulator   = False
        self.state       = np.array([position, lin_vel])

        self.satellite = satellite

        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)
        self.update_transformation_matrices()
        self.enviro      = reduced_environment


    # These are calculations and not really part of dynamics
    # earth sun vector
    # sun vector
    # relative_vel
    # magnetic_field ; environmental calculation

    # These are forces so make them part of dynamics
    # drag
    # gravity
    # grav_torque
    # forces

    def drag(self, v, rho=3.29e-12):
        '''
        Equation for drag. Assumes constant frontal area and atmospheric density for simplicity.

        Parameters
        ----------
        v : numpy.ndarray
            Velocity of satellite in inertial frame.
        rho : float
            Air density @400 km (per SMAD) kg/m^3

        Returns
        -------
        numpy.ndarray
            Drag force on satellite.
        '''
        v_norm   = np.linalg.norm(v)
        F        = -0.5 * rho * self.satellite.cd * self.satellite.area * v_norm * v
        return F

    def gravity(self, position):
        '''
        First order gravity model, per Newton.

        Parameters
        ----------
        x : numpy.ndarray
            Position of satellite in inertial frame.

        Returns
        -------
        numpy.ndarray
            Gravitational acceleration (m/s^2) of satellite.
        '''
        length = np.linalg.norm(position)
        coeff  = self.MU / length**3
        a      = -coeff * position
        return a

    def forces(self, x, v):
        '''
        Forces acting on satellite.

        Parameters
        ----------
         : numpy.ndarray
            Position of satellite in inertial frame.
        v : numpy.ndarray
            Velocity of satellite in inertial frame.

        Returns
        -------
        numpy.ndarray
            Total forces acting on satellite.
        '''
        v_rel = self.enviro.relative_vel(x, v)
        D = self.drag(v_rel)
        G = self.gravity(x) * self.satellite.mass
        return D + G
        


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

