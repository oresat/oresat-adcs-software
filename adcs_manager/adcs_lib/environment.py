import numpy as np
from adcs_lib import frame, quaternion

class ReducedEnvironment():
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self):
        self.AU    = 1.496e11 # m, 1 mean distance from earth to sun
        self.EARTH_ROTATION = 7.2921158553e-5 #: Rotation of Earth around its axis (rad/s).
        self.G_NEWTON = 6.6743e-11 #m^3 kg^-1 s^-2
        self.EARTH_RAD = 6.378137e6 # m, earth mean equitorial radius
        self.EARTH_MASS = 5.9733328e24 # kg, mass of earth including atmosphere
        self.MU = self.G_NEWTON * self.EARTH_MASS #: Gravitational parameter of satellite w.r.t. Earth.
        # ref markely & crassidis
        self.J2 = 1.08262668355e-3
        self.J3 = -2.53265648533e-6
        self.J4 = -1.61962159137e-6
        self.RHO = 3.29e-12 # kg/m^3 mean air density, @400 km (per SMAD)
        self.CD = 2 # satellite drag coeff
        self.A = 0.0166 # m^2, mean surface area
        self.M = 2.2 # kg, satellite mass

    def sun_vector(self, clock, x):
        '''More details on page 420 of Markely & Crassidis. For now assume the sun is infinitely far away.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        T_UT1      = (clock.julian_date(clock.hour, clock.minute, clock.second) - 2451545) / 36525
        mean_long  = (280.46 + 36000.771 * T_UT1) % 360 # degrees mean longitude
        mean_anom  = np.radians((357.5277233 + 35999.05034 * T_UT1) % 360) # rad mean anomaly
        ecl_long   = np.radians(mean_long + 1.914666471 * np.sin(mean_anom) + 0.019994643 * np.sin(2*mean_anom)) # ecliptic longitude
        ecl_oblq   = np.radians(23.439291 - 0.0130042 * T_UT1)
        earth_to_sun = np.array([np.cos(ecl_long),
                                np.cos(ecl_oblq) * np.sin(ecl_long),
                                np.sin(ecl_oblq) * np.sin(ecl_long)])
        S_inertial = self.AU * earth_to_sun - x
        return quaternion.normalize(S_inertial)

    # assume atmosphere rotates with earth
    # relative velocity of satellite with respect to atmosphere at location on earth
    # with respect to inertial frame and expressed in body frame
    def relative_vel(self, x, v):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        v_rel = np.array([v[0] + self.EARTH_ROTATION * x[1],
                          v[1] + self.EARTH_ROTATION * x[0],
                          v[2]])
        return v_rel # m/s

    # drag equation, unit agnostic
    def drag(self, v):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        v_norm   = np.linalg.norm(v)
        F        = -0.5 * self.RHO * self.CD * self.A * v_norm * v
        return F

    def gravity(self, position):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        length = np.linalg.norm(position)
        coeff  = self.MU / length**3
        F      = -coeff * position * self.M
        return F

    def forces(self, x, v):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        v_rel = self.relative_vel(x, v)
        D = self.drag(v_rel)
        G = self.gravity(x)
        return D + G

class Environment():
    '''Environmental models.'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self, satellite, hi_fi):
        self.hi_fi = hi_fi # high fidelity
        self.AU    = 1.496e11 # m, 1 mean distance from earth to sun
        self.EARTH_ROTATION = 7.2921158553e-5 #: Rotation of Earth around its axis (rad/s).
        self.G_NEWTON = 6.6743e-11 #m^3 kg^-1 s^-21.496
        self.EARTH_RAD = 6.378137e6 # m, earth mean equitorial radius
        self.EARTH_MASS = 5.9733328e24 # kg, mass of earth including atmosphere
        self.MU = self.G_NEWTON * self.EARTH_MASS #: Gravitational parameter of satellite w.r.t. Earth.
        # ref markely & crassidis
        self.J2 = 1.08262668355e-3
        self.J3 = -2.53265648533e-6
        self.J4 = -1.61962159137e-6
        # for geocentric magnetic field model
        # see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
        self.MAG_REF_RADIUS = 6.3712e6 # m, magnetic spherical ref. radius
        # these coefficients are in nT. from IGRF-13, 2020.0
        self.G_1_1 = -1450.9 # secular variation 7.4 / year
        self.H_1_1 = 4652.5 # secular variation -25.9 / year
        self.G_1_0 = -29404.8 # secular variation 5.7 / year
        self.m     = self.MAG_REF_RADIUS**3 * 1E-09 * np.array([self.G_1_1, self.H_1_1, self.G_1_0]) # magnetic dipole in ECEF
        self.satellite = satellite

    def sun_vector(self, clock, x):
        '''More details on page 420 of Markely & Crassidis. For now assume the sun is infinitely far away.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        T_UT1      = (clock.julian_date(clock.hour, clock.minute, clock.second) - 2451545) / 36525
        mean_long  = (280.46 + 36000.771 * T_UT1) % 360 # degrees mean longitude
        mean_anom  = np.radians((357.5277233 + 35999.05034 * T_UT1) % 360) # rad mean anomaly
        ecl_long   = np.radians(mean_long + 1.914666471 * np.sin(mean_anom) + 0.019994643 * np.sin(2*mean_anom)) # ecliptic longitude
        ecl_oblq   = np.radians(23.439291 - 0.0130042 * T_UT1)
        earth_to_sun = np.array([np.cos(ecl_long),
                                np.cos(ecl_oblq) * np.sin(ecl_long),
                                np.sin(ecl_oblq) * np.sin(ecl_long)])

        S_inertial = self.AU * earth_to_sun - x
        return quaternion.normalize(S_inertial)

    # exponentially decaying model atmosphere page 406
    # h is height above geode in m (i.e., in geodetic coordinates)
    def atmo_density(self, h):
        '''

        Parameters
        ----------

        Returns
        -------
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

    # assume atmosphere rotates with earth
    # relative velocity of satellite with respect to atmosphere at location on earth
    # with respect to inertial frame and expressed in body frame
    def relative_vel(self, x, v):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        v_rel = np.array([v[0] + self.EARTH_ROTATION * x[1],
                          v[1] + self.EARTH_ROTATION * x[0],
                          v[2]])
        return v_rel # m/s

    # drag equation, unit agnostic
    def drag(self, rho, v, attitude):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        v_norm   = np.linalg.norm(v)
        v_ref    = quaternion.sandwich(attitude, v / v_norm)
        (A, CoP) = self.satellite.area_and_cop(v_ref)
        F        = -0.5 * rho * self.satellite.drag_coeff * v_norm * v * A
        T        = np.cross(CoP, quaternion.sandwich(attitude, F))
        return np.array([F, T])

    # refer to Markely and Crassidis
    def hi_fi_gravity(self, position, r, coeff):
        '''

        Parameters
        ----------

        Returns
        -------
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

    def gravity(self, position, length, attitude):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        coeff  = self.MU / length**2
        if self.hi_fi:
            accel = self.hi_fi_gravity(position, length, coeff)
        else:
            accel = -coeff * position / length
        F      = accel * self.satellite.mass
        n      = quaternion.sandwich(attitude, -position / length)
        T      = 3 * coeff / length * np.cross(n, self.satellite.total_moment.dot(n))
        return np.array([F, T])


    # dipole model
    # gradient of 1st order term of IGRF model of magnetic potential
    # see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
    # be aware that every year, these approximated coefficients change a little
    # see page 403 - 406 for details. i expect 20-50 uT magnitude
    def magnetic_field(self, r_ecef, length, GCI_to_ECEF):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        R      = (3 * np.dot(self.m, r_ecef) * r_ecef - self.m * length**2) / length**5 # nT
        B      = GCI_to_ECEF.T.dot(R)
        return B

    def env_F_and_T(self, position, velocity, attitude, GCI_to_ECEF, mag_moment):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        r_ecef       = GCI_to_ECEF.dot(position)
        length       = np.linalg.norm(position)
        B            = self.magnetic_field(r_ecef, length, GCI_to_ECEF)
        B_body       = quaternion.sandwich(attitude, B)
        lat, long, h = frame.ecef_to_lla(r_ecef)
        #lat, long, h = frame.ECEF_to_geodetic(r_ecef)
        rho          = self.atmo_density(h)
        v_rel        = self.relative_vel(position, velocity)

        D = self.drag(rho, v_rel, attitude)
        G = self.gravity(position, length, attitude)
        M = np.array([np.zeros(3), np.cross(mag_moment, B_body)])
        return D + G + M
