import numpy as np

from ..functions import vector

'''This module contains classes to represent and control the physical hardware of a satellite.'''



class SensitiveInstrument():
    '''
    This class represents an object that cares about where it is pointed, such as a camera or an antenna.

    Parameters
    ----------
    boresight : numpy.ndarray
        3d array indicating pointing direction in body-coordinates.
    bound : float
        Inclusion/exclusion angle (degrees) for instrument.
    forbidden : bool
        True if the instrument is forbidden from getting closer than the exclusion angle.
        False if the instrument is required to stay within the inclusion angle.
    '''
    def __init__(self, boresight, bounds, forbidden, obj_ids):
        self.sign      = [-1.0 if val else 1.0 for val in forbidden]
        self.bounds    = [np.radians(val) for val in bounds]
        self.cos_psi   = [np.cos(bound) for bound in self.bounds]
        self.boresight = boresight
        self.obj_ids   = obj_ids

    def set_gains(self, stop, gains):
        self.sigma     = [0.5 * np.abs(np.cos(bound - self.sign[j] * stop) - self.cos_psi[j]) for j, bound in enumerate(self.bounds)]
        self.iota      = [0.25 * sig for sig in self.sigma]
        self.gains     = [val for val in gains]
        self.push      = [val * 0.5 for val in self.gains]

    def cares_about(self, i, psi):
        return i in self.obj_ids and psi < self.sigma[i]

class Magnetorquer():
    '''
    Represents one magnetorquer of either type. Uses latest data given by the Torqueducks.
    Linearized torque rods are a fine model below 0.5 A input.

    Parameters
    ----------
    type : string
        "Square", "Rod", or "LinearRod", depending on which type we are modeling.
    axis : numpy.ndarray
        3d array of dipole direction (for positive current) in body-coordinates.
    max_A : float
        Maximum current (A) that this magnetorquer is allowed to use.
    '''
    def __init__(self, type, axis, max_A):
        self.type  = type
        self.max_A = max_A
        self.axis  = axis
         # R is resistance in Ohms.
        if   type == "Square":
            self.R = 14.9
        elif type == "Rod":
            self.R = 6.28
        elif type == "LinearRod":
            self.R = 6.28

        # maximum magnetic moment that can be induced
        self.max_m = np.linalg.norm(self.actuate(max_A))

    def actuate(self, I):
        '''
        Actuator model for magnetorquer.

        Parameters
        ----------
        I : float
            Effective input current (A).

        Returns
        -------
        numpy.ndarray
            Magnetic moment induced by current (A m^2)
        '''

        # IMO, set a lambda function and then have this function call that lambda function

        if self.type == "Square":
            m = self.axis * I * 1.584 # 200 * (89e-3)**2 m^2, turns times area
        elif self.type == "Rod":
            m = self.axis * (2.36 * abs(I) - 1.1 * I**2) * np.sign(I)
        elif self.type == "LinearRod":
            m = self.axis * (1.87 * I)

        return m

    def getCurrent(self, m):
        '''
        Control commands for magnetorquer.

        Parameters
        ----------
        m : numpy.ndarray
            3d array for commanded magnetic moment (A m^2).

        Returns
        -------
        float
            Current (A) required to induce commanded magnetic moment.
        '''
        if   self.type == "Square":
            I = m * 0.631313
        elif self.type == "Rod":
            I = np.sign(m) * 0.0181818 * (59 - np.sqrt(3481 - 2750 * abs(m))) # thanks wolfram alpha
        elif self.type == "LinearRod":
            I = m * 0.5347593

        return I

    def power(self, I):
        '''
        Power consumption.

        Parameters
        ----------
        I : float
            Input current (A).

        Returns
        -------
        float
            Power (W) to drive magnetorquer.
        '''
        return self.R * I**2


class MagnetorquerSystem():
    '''
    Represents the entire subsystem of magnetorquers, for convenience.
    Assumes the z-axis torquer is a square coil and the x and y torquers are rods.

    Parameters
    ----------
    linearized : bool
        True if the torque rods are linearized around a limited region of current. False for saturating model.
    max_A_sys : float
        Maximum current (A) available for the entire system to use.
    '''
    def __init__(self, torquers):
        self.torquers = torquers

    def distribute(self, m):
        '''
        Distributes a commanded magnetic moment to commanded currents.
        Assumes the z-torquer has least maximum current, and prevents torquers from exceeding their maximums.

        Parameters
        ----------
        m : numpy.ndarray
            Commanded magnetic moment (A m^2).

        Returns
        -------
        numpy.ndarray
            3d array for x, y, and z currents (A) needed to induce the magnetic moment.
        '''

        # For some reason this did not saturate to the a maximum
        #if abs(m[2]) > self.torquers[2].max_m:
        #    m = m * self.torquers[2].max_m / abs(m[2])
        #m     = vector.saturation(m, self.torquers[0].max_m)
        
        # scale down the vector so that no magnetorquer is saturated.
        factor = min([torquer.max_m / m[index] for index,torquer in enumerate(self.torquers)])
        if factor < 1:
            m = m*factor

        return np.array([torquer.getCurrent(m[i]) for i, torquer in enumerate(self.torquers)])

    def actuate(self, I):
        '''
        Actuator model for system.

        Parameters
        ----------
        I : numpy.ndarray
            Input x, y, and zcurrents (A).

        Returns
        -------
        numpy.ndarray
            Total induced magnetic moment.
        '''
        return sum([torquer.actuate(I[j]) for j, torquer in enumerate(self.torquers)])

    def power(self, I):
        '''
        Power consumption.

        Parameters
        ----------
        I : numpy.ndarray
            Input currents (A).

        Returns
        -------
        float
            Power (W) to drive entire magnetorquer subsystem.
        '''
        return sum([torquer.power(I[j]) for j, torquer in enumerate(self.torquers)])

class Wheel():
    '''
    Represents a single reaction wheel.

    Parameters
    ----------
    inclination : float
        Angle of reaction wheel axis above XY plane
    azimuth : float
        Angle of reaction wheel axis from X-Y axes
    parallel_moment : float
        Axial moment of reaction wheel.
    orthogonal_moment : float
        Transverse moment of reaction wheel.
    index : int
        Order of reaction wheel proceeding clockwise from x-axis.
    '''
    def __init__(self, axis, parallel_moment, orthogonal_moment):
        self.axis = axis
        self.par_mom_scalar    = parallel_moment
        self.orth_mom_scalar   = orthogonal_moment

        outer                  = np.outer(self.axis, self.axis)
        self.parallel_moment   = parallel_moment   * outer
        self.orthogonal_moment = orthogonal_moment * (np.identity(3) - outer)


    def acceleration(self, commanded_torque):
        '''
        Wheel angular acceleration produced by commanded torque, modulo wheel dynamics.

        Parameters
        ----------
        commanded_torque : float
            Commanded torque (Nm).

        Returns
        -------
        float
            Produced acceleration (rad/s^2).
        '''
        a = commanded_torque / self.par_mom_scalar

        return a

    def momentum(self, velocity):
        '''
        Angular momentum of wheel in body-coordinates, including contribution from body's velocity around wheel axis.

        Parameters
        ----------
        velocity : float
            Present wheel velocity.
        ang_vel : numpy.ndarray
            Angular velocity of satellite.

        Returns
        -------
        numpy.ndarray
            Angular momentum of wheel (kg rad/s).
        '''
        return self.axis * velocity * self.par_mom_scalar

    def torque(self, accl):
        '''
        Torque of wheel in body-coordinates, including contribution from body's acceleration around wheel axis.

        Parameters
        ----------
        accl : float
            Present wheel acceleration.
        ang_accl : numpy.ndarray
            Angular acceleration of satellite.

        Returns
        -------
        numpy.ndarray
            Torque of wheel (Nm).
        '''
        return self.axis * accl * self.par_mom_scalar

class ReactionWheelSystem():
    '''
    Represents the whole system of reaction wheels, mostly for convenience.
    Note that the null space of the given configuration is spanned by [-1, 1, -1, 1]^T.

    Parameters
    ----------
    inclination : float
        Angle of reaction wheel axis above XY plane
    azimuth : float
        Angle of reaction wheel axis from X-Y axes
    parallel_moment : float
        Axial moment of reaction wheel.
    orthogonal_moment : float
        Transverse moment of reaction wheel.
    max_T : float
        Maximum torque any given wheel can produce.
    torque_limited : bool
        True if reaction wheels are limited by the amount of torque they can produce.
    '''
    def __init__(self, wheels, max_T, torque_limited):
        self.torque_limited    = torque_limited
        self.max_T             = max_T

        self.wheels = wheels

        # recalculate parameters
        self.axes              = np.array([wheel.axis for wheel in self.wheels]).T
        # the pseudoinverse method minimizes L2 norm of torque/momentum vector (which is sum of individual wheels)
        # there is another method for minimizing max effort of any wheel (minimax)
        # right now we don't care, but we may later
        #: Distributes torque of whole wheel system into torque of individual wheels.
        # the distribution should be the pseudo inverse of the axes
        self.distribution      = np.linalg.pinv(self.axes)
        self.parallel_moment   = sum([wheel.parallel_moment for wheel in self.wheels])
        self.orthogonal_moment = sum([wheel.orthogonal_moment for wheel in self.wheels])
        self.inv_par_moment    = np.linalg.inv(self.parallel_moment)



    def accelerations(self, commanded_torque):
        '''
        Wheel angular accelerations produced by commanded torque, modulo wheel dynamics.
        If system is torque limited, makes sure accelerations are within bounds.

        Parameters
        ----------
        commanded_torque : numpy.ndarray
            Commanded torque (N m).

        Returns
        -------
        numpy.ndarray
            Produced acceleration (rad/s^2).
        '''
        T     = self.distribution.dot(commanded_torque)
        if self.torque_limited:
            T = vector.saturation(T, self.max_T)

        return np.array([wheel.acceleration(T[i]) for i, wheel in enumerate(self.wheels)])

    def momentum(self, velocities):
        '''Angular momentum of all wheels together along their respective spin axes in the body frame,
        including the body's momentum around said spin axes.

        Parameters
        ----------
        velocity : numpy.ndarray
            Present wheel velocity.
        ang_vel : numpy.ndarray
            Angular velocity of satellite.

        Returns
        -------
        numpy.ndarray
            Angular momentum of wheel system (kg rad/s).
        '''
        return sum([wheel.momentum(velocities[i]) for i , wheel in enumerate(self.wheels)])

    def torque(self, accls):
        '''Torque of wheel system in body-coordinates, including contribution from body's acceleration around wheel axes.
        Assumes acceleration command is instantaneous, at some point, we will need to either model internal wheel dynamics or compensate for them.

        Parameters
        ----------
        accls : numpy.ndarray
            Present wheel acceleration.
        ang_accl : numpy.ndarray
            Angular acceleration of satellite.

        Returns
        -------
        numpy.ndarray
            Torque of wheel system (N m).
        '''
        return sum([wheel.torque(accls[i]) for i, wheel in enumerate(self.wheels)])

class Wall():
    '''One of the satellite's walls. The purpose of this is for aerodynamic drag (and eventually solar radiation pressure).

    Parameters
    ----------
    distance : float
        Distance (m) from origin.
    normal : numpy.ndarray
        Surface normal vector.
    length : float
        Length (m) of wall.
    width : float
        Width (m) of wall.
    absorption : float
        Absorption coefficient of surface.
    '''
    def __init__(self, distance, normal, length, width, absorption):
        self.distance   = distance
        self.normal     = normal
        self.length     = length
        self.width      = width
        self.area       = length * width
        self.centroid   = normal * distance
        self.absorption = absorption
        self.reflection = 1 - absorption

    def projected_area(self, v_ref):
        '''Projected surface area as function of reference unit vector.
        0 in the case that the surface faces away from the reference vector.

        Parameters
        ----------
        v_ref : numpy.ndarray
            Vector defining the plane the surface is projected onto.

        Returns
        -------
        float
            Projected surface area.
        '''
        c_beta = max(0, np.dot(self.normal, v_ref))

        return c_beta * self.area

    def center_of_pressure(self, v_ref):
        '''Exposed area and area-weighted centroid.

        Parameters
        ----------
        v_ref : numpy.ndarray
            Vector defining the plane the surface is projected onto.

        Returns
        -------
        numpy.ndarray
            First entry is the projected surface area, second is the center of pressure vector.
        '''
        A  = self.projected_area(v_ref)
        cp = A * self.centroid

        return np.array([A, cp], dtype=object)

    def srp_force(self, SRP, S):
        '''Calculates solar radiation pressure torque on this wall.
        Assumes there is no diffuse reflection.

        Parameters
        ----------
        SRP : float
            Solar radiation pressure.
        S : numpy.ndarray
            Sun vector in body coordinates.

        Returns
        -------
        numpy.ndarray
            Force (N) and torque (N m) on satellite.
        '''
        c_beta = np.dot(self.normal, S)
        A, cp  = self.center_of_pressure(S)
        F      = - SRP * A * (2 * self.reflection * c_beta * self.normal + self.absorption * S)
        T      = np.cross(cp, F)
        return np.array([F, T])

    def drag_force(self, drag_pressure, v):
        '''Calculates solar radiation pressure torque on this wall.
        Assumes there is no diffuse reflection.

        Parameters
        ----------
        SRP : float
            Solar radiation pressure.
        S : numpy.ndarray
            Sun vector in body coordinates.

        Returns
        -------
        numpy.ndarray
            Force (N) and torque (N m) on satellite.
        '''
        c_beta = np.dot(self.normal, v)
        A, cp  = self.center_of_pressure(v)
        F      = drag_pressure * A * v
        T      = np.cross(cp, F)
        return np.array([F, T])


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
    def __init__(self, mass, dimensions, absorption, drag_coeff, principal_moments, product_moments, reduced, rw_sys, mt_sys, sensitive_instruments=[]):
        
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


        # Define the reaction wheels
        self.reaction_wheels = rw_sys
        # define the magnetorquers
        self.magnetorquers = mt_sys
        # define the sensitive instruments
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

    def srp_forces(self, SRP, S):
        '''Calculates solar radiation pressure torque on this wall.
        Assumes there is no diffuse reflection.

        Parameters
        ----------
        SRP : float
            Solar radiation pressure.
        S : numpy.ndarray
            Sun vector in body coordinates.

        Returns
        -------
        numpy.ndarray
            Force (N) and torque (N m) on satellite.
        '''
        F_and_T = sum([wall.srp_force(SRP, S) for wall in self.walls])
        return F_and_T

    def drag_forces(self, drag_pressure, v):
        '''Calculates solar radiation pressure torque on this wall.
        Assumes there is no diffuse reflection.

        Parameters
        ----------
        SRP : float
            Solar radiation pressure.
        S : numpy.ndarray
            Sun vector in body coordinates.

        Returns
        -------
        numpy.ndarray
            Force (N) and torque (N m) on satellite.
        '''
        F_and_T = sum([wall.drag_force(drag_pressure, v) for wall in self.walls])
        return F_and_T
