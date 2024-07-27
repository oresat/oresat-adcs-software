import numpy as np
from ..functions import frame, quaternion, vector

from .env3 import ReducedEnvironment



class Environment(ReducedEnvironment):
    '''Environmental models for sun, aerodynamics, magnetic field, and gravity.

    Parameters
    ----------
    satellite : structure.Satellite
        Object representing the satellite.
    config: dict
        Configuration dictionary of dictionaries, please see examples
        Example: {"EARTH_RAD": {"value": 6.378137e6, "desc": "m, earth mean equitorial radius"}}
    hi_fi : bool
        True if using higher order terms of gravity model. False if using first order approximation.
    '''
    def __init__(self, satellite, config, hi_fi):
        super().__init__(satellite, config)

        self.SRP_coeff = 1362 * self.AU**2 / 299792458 # Newtons, eq D.56
        self.hi_fi = hi_fi # high fidelity

    def sun_vector(self, clock, x, q):
        '''Unit vector in inertial coordinates pointing from satellite to the sun.
        More details on page 420 of Markely & Crassidis. For now assume the sun is a constant distance away.

        Parameters
        ----------
        clock : jday.Clock
            Clock is needed because the position of the sun depends on the date and time (obviously).
        x : numpy.ndarray
            Position of the satellite in inertial frame.
        q : numpy.ndarray
            Attitude of satellite.

        Returns
        -------
        numpy.ndarray
            Inertial unit vector pointing at sun from satellite.
        '''
        # earth_sun_vector exists in parent class

        earth_to_sun = self.earth_sun_vector(clock)
        # remove stuff above this line: reduced environment will hvae an earth_sun_vector function
        
        sat_to_sun   = self.AU * earth_to_sun - x
        S_inertial   = vector.normalize(sat_to_sun)

        in_shadow    = np.dot(x, earth_to_sun) < - np.sqrt(np.dot(x, x) - self.EARTH_RAD**2) # cylindrical approx for earth shadowing

        SRP          = self.SRP_coeff / np.dot(sat_to_sun, sat_to_sun) # solar radiation pressure

        if in_shadow:
            solar_F_and_T = np.array([np.zeros(3), np.zeros(3)])
        else:
            solar_F_and_T = self.satellite.srp_forces(SRP, quaternion.sandwich(q, S_inertial))
            solar_F_and_T[0] = quaternion.sandwich_opp(q, solar_F_and_T[0])

        return S_inertial, in_shadow, solar_F_and_T

    def atmo_density(self, h):
        '''
        Exponentially decaying atmosphere model. Refer to page 406 of Markely & Crassidis.
        Eventually we may want a higher fidelity model.
        Be aware that this is undefined outside of 250 - 500 km.

        Parameters
        ----------
        h : float
            Height (m) above geode in geodetic coordinates.

        Returns
        -------
        float
            Atmospheric density (kg/m^3).
        '''
        if 250000 <= h and h < 300000:
            h_0   = 250000 # m
            rho_0 = 7.248 * 10**(-11) # kg/m^3
            H     = 46900 # m
        elif 300000 <= h and h < 350000:
            h_0   = 300000 # m
            rho_0 = 2.418 * 10**(-11) # kg/m^3
            H     = 52500 # m
        elif 350000 <= h and h < 400000:
            h_0   = 350000 # m
            rho_0 = 9.158 * 10**(-12) # kg/m^3
            H     = 56400 # m
        elif 400000 <= h and h < 450000:
            h_0   = 400000 # m
            rho_0 = 3.727 * 10**(-12) # kg/m^3
            H     = 59400 # m
        elif 450000 <= h and h < 500000:
            h_0   = 400000 # m
            rho_0 = 1.585 * 10**(-12) # kg/m^3
            H     = 62200 # m
        else:
            print("height out of bounds!", h) # when less lazy, allow for decaying orbit
        return rho_0 * np.exp((h_0 - h) / H) # kg/m^3

    # relative_vel exists in parent class

    # drag equation, unit agnostic
    def drag(self, rho, v, attitude):
        '''
        Equation for drag taking satellite altitude and orientation into account.

        Parameters
        ----------
        rho : float
            Atmospheric density.
        v : numpy.ndarray
            Velocity of satellite in inertial frame.
        attitude : numpy.ndarray
            Attitude of satellite.

        Returns
        -------
        numpy.ndarray
            Array of arrays for drag force and torque on satellite.
        '''
        v_norm   = np.linalg.norm(v)
        v_ref    = quaternion.sandwich(attitude, v / v_norm)
        pressure = -0.5 * rho * self.satellite.drag_coeff * v_norm**2
        F_and_T  = self.satellite.drag_forces(pressure, v_ref)
        F_and_T[0] = quaternion.sandwich_opp(attitude, F_and_T[0])
        print('F_and_T: ', F_and_T)
        return F_and_T

    def hi_fi_gravity(self, position, r, coeff):
        '''
        Higher order gravity model including J2, J3, and J4 zonal terms.
        Refer to Markely and Crassidis.

        Parameters
        ----------
        position : numpy.ndarray
            Position of satellite in inertial frame.
        r : float
            Norm of position vector.
        coeff : float
            For computational convenience, mu / r^2.

        Returns
        -------
        numpy.ndarray
            Gravitational acceleration.
        '''
        xoverr = position[0] / r
        yoverr = position[1] / r
        zoverr = position[2] / r
        zoverrsquare = zoverr**2
        zoverrcube = zoverr*zoverrsquare
        zoverr4th = zoverrsquare**2
        rearthoverr = self.EARTH_RAD / r
        a   = position / r
        aj2 = 3/2 * self.J2 * rearthoverr**2 * np.array([(1 - 5 * zoverrsquare) * xoverr,
                                                          (1 - 5 * zoverrsquare) * yoverr,
                                                          (3 - 5 * zoverrsquare) * zoverr])
        aj3 = 1/2 * self.J3 * rearthoverr**3 * np.array([5 * (7 * zoverrcube - 3 * zoverr) * xoverr,
                                                          5 * (7 * zoverrcube - 3 * zoverr) * yoverr,
                                                          3 * (10 * zoverrsquare - 35/3 * zoverr4th - 1)])
        aj4 = 5/8 * self.J4 * rearthoverr**4 * np.array([(3 - 42 * zoverrsquare + 63 * zoverr4th) * xoverr,
                                                          (3 - 42 * zoverrsquare + 63 * zoverr4th) * yoverr,
                                                          -(15 - 70 * zoverrsquare + 63 * zoverr4th) * zoverr])
        return - coeff * (a + aj2 + aj3 + aj4)

    def gravity(self, position, attitude):
        '''
        Gravitational forces and torques.
        Note that gravity torque is fairly predictable and we could even use it for feedforward in controls.

        Parameters
        ----------
        position : numpy.ndarray
            Position of satellite in inertial frame.
        length : float
            Norm of position vector.
        attitude : numpy.ndarray
            Attitude of satellite.

        Returns
        -------
        numpy.ndarray
            Array of arrays for gravitational acceleration and torque.
        '''
        length=np.linalg.norm(position)
        coeff  = self.MU / length**2
        if self.hi_fi:
            accel = self.hi_fi_gravity(position, length, coeff)
        else:
            accel = -coeff * position / length
        n      = quaternion.sandwich(attitude, -position / length)
        T      = 3 * coeff / length * np.cross(n, self.satellite.total_moment.dot(n))
        return np.array([accel, T])

    # magnetic_field function in parent

    def env_F_and_T(self, position, velocity, attitude, clock, GCI_to_ECEF, mag_moment):
        '''
        External forces and torques acting on satellite.

        Parameters
        ----------
        position : numpy.ndarray
            Position of satellite in inertial frame.
        velocity : numpy.ndarray
            Velocity of satellite in inertial frame.
        attitude : numpy.ndarray
            Attitude of satellite.
        GCI_to_ECEF : numpy.ndarray
            Matrix for coordinate transformation from inertial frame to ECEF frame.
        mag_moment : numpy.ndarray
            Magnetic dipole moment of satellite.

        Returns
        -------
        numpy.ndarray
            Array of arrays for all forces and torques acting on satellite.
        '''
        r_ecef       = GCI_to_ECEF.dot(position)
        length       = np.linalg.norm(position)
        B            = self.magnetic_field(r_ecef, GCI_to_ECEF)
        B_body       = quaternion.sandwich(attitude, B)
        if self.hi_fi:
            lat, long, h = frame.ecef_to_lla(r_ecef)
            rho          = self.atmo_density(h)
            _, _, srp    = self.sun_vector(clock, position, attitude)
        else:
            rho          = 3.29e-12 # typical at 400 km
            srp          = np.array([np.zeros(3), np.zeros(3)])

        v_rel        = self.relative_vel(position, velocity)

        D = self.drag(rho, v_rel, attitude)
        G = self.gravity(position, attitude)
        G[0] *= self.satellite.mass
        M = np.array([np.zeros(3), np.cross(mag_moment, B_body)])
        #print('D: ', D)
        #print('G: ', G)
        #print('M: ', M)
        #print('srp: ', srp)
        return D + G + M + srp
