import numpy as np
from . import structure


class Satellite():
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
    def __init__(self, mass, dimensions, absorption, drag_coeff, principal_moments, product_moments, reduced, sensors, rw_sys, mt_sys, sensitive_instruments=[]):
        
        # The satellite should define its own dimensions and walls
        self.length         = dimensions[0]
        self.width          = dimensions[1]
        self.height         = dimensions[2]
        self.walls          = (structure.Wall(self.length/2, np.array([1, 0, 0]), self.width, self.height, absorption),
                               structure.Wall(self.length/2, np.array([-1, 0, 0]), self.width, self.height, absorption),
                               structure.Wall(self.width/2, np.array([0, 1, 0]), self.length, self.height, absorption),
                               structure.Wall(self.width/2, np.array([0, -1, 0]), self.length, self.height, absorption),
                               structure.Wall(self.height/2, np.array([0, 0, 1]), self.width, self.length, absorption),
                               structure.Wall(self.height/2, np.array([0, 0, -1]), self.width, self.length, absorption))

        
        #: Estimated drag coefficient.
        self.drag_coeff      = drag_coeff
        self.mass            = mass
        
        #self.enviro = environment

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
        
        
        
        # No arrays, maybe a dict or an object
        #self.state       = np.array([position, lin_vel, attitude, body_ang_vel, wheel_vel], dtype=object)
        #self.init_date   = date_and_time
        #year, month, day, hour, minute, second = date_and_time
        #self.clock       = jday.JClock(year, month, day, hour, minute, second)


        # just call frame or make it part of the environment
        # self.update_transformation_matrices()lass Satellite():

