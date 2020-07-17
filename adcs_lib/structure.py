import numpy as np

'''This module contains the physical constants for OreSat.'''

# MKS units
ORESAT_MASS = (21.66*4 + 2419.56) / 1000
INCLINATION = np.pi / 3
AZIMUTH     = np.pi / 4
PARALLEL    = 1.64023e-6
ORTHOGONAL  = 1.02562e-6
LENGTH      = 0.1
WIDTH       = 0.1
HEIGHT      = 0.2
PRINCIPAL   = np.array([1.378574142e-2,
                        1.378854578e-2,
                        5.49370596e-3])

class Wheel():
    '''A reaction wheel.'''

    def __init__(self, inclination, azimuth, parallel_moment, orthogonal_moment, index):
        self.inclination       = inclination
        self.azimuth           = azimuth
        self.index             = index
        self.par_mom_scalar    = parallel_moment
        self.orth_mom_scalar   = orthogonal_moment
        s_inc     = np.sin(inclination)
        angle     = azimuth + index * np.pi/2
        self.axis = np.array([s_inc * np.cos(angle),
                               s_inc * np.sin(angle),
                               np.cos(inclination)])

        outer = np.outer(self.axis, self.axis)
        self.parallel_moment   = parallel_moment * outer
        self.orthogonal_moment = orthogonal_moment * (np.identity(3) - outer)

    def acceleration(self, commanded_torque):
        a = commanded_torque / self.par_mom_scalar
        return a

    def momentum(self, velocity, ang_vel):
        return self.axis * (velocity + np.dot(ang_vel, self.axis)) * self.par_mom_scalar

    def torque(self, accl, ang_accl):
        return self.axis * (accl + np.dot(ang_accl, self.axis)) * self.par_mom_scalar

class ReactionWheelSystem():
    '''A system of reaction wheels.'''

    def __init__(self, inclination, azimuth, parallel_moment, orthogonal_moment):
        self.wheels            = [Wheel(inclination, azimuth, parallel_moment, orthogonal_moment, i) for i in range(4)]
        #: Spin axes of reaction wheels.
        self.axes              = np.array([wheel.axis for wheel in self.wheels]).T
        # the pseudoinverse method minimizes L2 norm of torque/momentum vector (which is sum of individual wheels)
        # there is another method for minimizing max effort of any wheel (minimax)
        # right now we don't care, but we may later
        #: Distributes torque of whole wheel system into torque of individual wheels.
        self.distribution      = np.linalg.pinv(self.axes)
        self.parallel_moment   = sum([wheel.parallel_moment for wheel in self.wheels])
        self.orthogonal_moment = sum([wheel.orthogonal_moment for wheel in self.wheels])
        self.inv_par_moment    = np.linalg.inv(self.parallel_moment)

    def accelerations(self, commanded_torque):
        T = self.distribution.dot(commanded_torque)
        return np.array([wheel.acceleration(T[i]) for i, wheel in enumerate(self.wheels)])

    def momentum(self, velocities, ang_vel):
        '''Angular momentum of all wheels together (along their respective spin axes), body referenced.'''
        return sum([wheel.momentum(velocities[i], ang_vel) for i , wheel in enumerate(self.wheels)])

    def torque(self, accls, ang_accl):
        '''Torque of wheel system (body referenced), assuming acceleration command is instantaneous.
        At some point, we will need to either model internal wheel dynamics or compensate for them'''
        return sum([wheel.torque(accls[i], ang_accl) for i, wheel in enumerate(self.wheels)])

class Wall():
    '''One of the satellite's walls.'''
    def __init__(self, distance, normal, length, width):
        self.distance = distance
        self.normal   = normal
        self.length   = length
        self.width    = width
        self.area     = length*width
        self.centroid = normal*distance

    def projected_area(self, v_ref):
        '''Projected surface area as function of reference vector'''
        c_beta = max(0, np.dot(self.normal, v_ref))
        return c_beta * self.area

    def center_of_pressure(self, v_ref):
        '''Exposed area and area-weighted centroid, we ignore shading.'''
        A = self.projected_area(v_ref)
        cp = A * self.centroid
        return np.array([A, cp], dtype=object)

class Satellite():
    '''A rectangular satellite and its material properties'''

    def __init__(self, length, width, height, principal_moments,
                inclination, azimuth, parallel_moment, orthogonal_moment):
        self.length = length
        self.width  = width
        self.height = height
        self.walls  = (Wall(length/2, np.array([1, 0, 0]), width, height),
                       Wall(length/2, np.array([-1, 0, 0]), width, height),
                       Wall(width/2, np.array([0, 1, 0]), length, height),
                       Wall(width/2, np.array([0, -1, 0]), length, height),
                       Wall(height/2, np.array([0, 0, 1]), width, length),
                       Wall(height/2, np.array([0, 0, -1]), width, length))
        #: Estimated drag coefficient.
        self.drag_coeff = 2 # could be anywhere from 1 - 2.5
        #: Mass of OreSat in kg
        self.mass = ORESAT_MASS
        self.reaction_wheels = ReactionWheelSystem(inclination, azimuth, parallel_moment, orthogonal_moment)
        #: Moment of inertia for the satellite except the moments of wheels about spin axes.
        self.reduced_moment  = np.diag(principal_moments) + self.reaction_wheels.orthogonal_moment
        #: Moment of inertia for reaction wheels about spin axes with respect to principal axes.
        self.wheel_moment    = self.reaction_wheels.parallel_moment
        #: Total moment of inertia of the satellite.
        self.total_moment    = self.reduced_moment + self.wheel_moment
        #: Inverse of the moment of inertia for the satellite except the moments of wheels about spin axes.
        self.inv_red_moment  = np.linalg.inv(self.reduced_moment)

    def area_and_cop(self, v_ref):
        '''Projected surface area and center of pressure (ignoring shading).
        Note that I (Cory) don't have a great deal of confidence in this CoP calculation.'''
        (A, CoP) = sum([wall.center_of_pressure(v_ref) for wall in self.walls])
        return (A, CoP/A)
