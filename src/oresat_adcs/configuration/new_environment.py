import numpy as np
from ..functions import frame, quaternion, vector







class Environment():
    AU    = 1.496e11 # m, 1 mean distance from earth to sun
    EARTH_ROTATION = 7.2921158553e-5 #: Rotation of Earth around its axis (rad/s).
    G_NEWTON = 6.6743e-11 #m^3 kg^-1 s^-2
    EARTH_RAD = 6.378137e6 # m, earth mean equitorial radius
    EARTH_MASS = 5.9733328e24 # kg, mass of earth including atmosphere
    
    # ref markely & crassidis
    J2 = 1.08262668355e-3
    J3 = -2.53265648533e-6
    J4 = -1.61962159137e-6
    
    # for geocentric magnetic field model
    # see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
    MAG_REF_RADIUS = 6.3712e6 # m, magnetic spherical ref. radius
    # these coefficients are in nT. from IGRF-13, 2020.0
    G_1_1 = -1450.9 # secular variation 7.4 / year
    H_1_1 = 4652.5 # secular variation -25.9 / year
    G_1_0 = -29404.8 # secular variation 5.7 / year

    '''
    Simplified environmental models for sun, aerodynamics, and gravity.
    '''
    def __init__(self, hi_fi=False):
        self.hi_fi = hi_fi
        # Until I calcualte these, just keep them here
        self.MU = self.G_NEWTON * self.EARTH_MASS #: Gravitational parameter of satellite w.r.t. Earth.
        self.m     = self.MAG_REF_RADIUS**3 * 1E-09 * np.array([self.G_1_1, self.H_1_1, self.G_1_0]) # magnetic dipole in ECEF
        self.SRP_coeff = 1362 * self.AU**2 / 299792458 # Newtons, eq D.56


    def relative_vel(self, x, v):
        '''Relative velocity of satellite, with respect to the atmosphere at this location,
        assuming atmosphere rotates with earth with respect to inertial frame.

        Parameters:
            x (numpy.ndarray): Position of satellite in inertial frame.
            v (numpy.ndarray): Velocity of satellite in inertial frame.

        Returns:
            numpy.ndarray : Relative velocity of satellite.
        '''
        v_rel = np.array([v[0] + self.EARTH_ROTATION * x[1],
                          v[1] + self.EARTH_ROTATION * x[0],
                          v[2]])
        return v_rel # m/s


    def SRP_info(self, clock, x):
        '''Unit vector in inertial coordinates pointing from satellite to the sun.
        More details on page 420 of Markely & Crassidis. For now assume the sun is a constant distance away.

        Parameters:
            clock (jday.Clock): Clock is needed because the position of the sun depends on the date and time (obviously).
            x (numpy.ndarray): Position of the satellite in inertial frame.

        Returns:
            float: Solar radiation pressure (N/m^2)
            bool: True if position is in earth shadow
            numpy.ndarray: normalized inertial unit vector pointing at the sun from the satellite
        '''
        T_UT1      = (clock.julian_date() - 2451545) / 36525
        mean_long  = (280.46 + 36000.771 * T_UT1) % 360 # degrees mean longitude
        mean_anom  = np.radians((357.5277233 + 35999.05034 * T_UT1) % 360) # rad mean anomaly
        ecl_long   = np.radians(mean_long + 1.914666471 * np.sin(mean_anom) + 0.019994643 * np.sin(2*mean_anom)) # ecliptic longitude
        ecl_oblq   = np.radians(23.439291 - 0.0130042 * T_UT1)
        
        earth_to_sun = np.array([np.cos(ecl_long),
                                np.cos(ecl_oblq) * np.sin(ecl_long),
                                np.sin(ecl_oblq) * np.sin(ecl_long)])

        sat_to_sun   = self.AU * earth_to_sun - x

        S_inertial = vector.normalize(sat_to_sun)
        in_shadow    = np.dot(x, earth_to_sun) < - np.sqrt(np.dot(x, x) - self.EARTH_RAD**2) # cylindrical approx for earth shadowing

        SRP          = self.SRP_coeff / np.dot(sat_to_sun, sat_to_sun) # solar radiation pressure
        return SRP, in_shadow, S_inertial 



    def atmo_density(self, h):
        '''Exponentially decaying atmosphere model. Refer to page 406 of Markely & Crassidis.
        Eventually we may want a higher fidelity model.
        Be aware that this is undefined outside of 250 - 500 km.

        Parameters:
            h (float): Height (m) above geode in geodetic coordinates.

        Returns:
            float: Atmospheric density (kg/m^3).
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


    def hi_fi_gravity(self, position, r, coeff):
        '''Higher order gravity model including J2, J3, and J4 zonal terms.
        Refer to Markely and Crassidis.

        Parameters:
            position (numpy.ndarray): Position of satellite in inertial frame.
            r (float): Norm of position vector.
            coeff (float) For computational convenience, mu / r^2.

        Returns:
            numpy.ndarray: Gravitational acceleration.
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


    def gravity_accel(self, position, length):
        '''Gravitational forces and torques.
        Note that gravity torque is fairly predictable and we could even use it for feedforward in controls.

        Parameters: 
            position (numpy.ndarray): Position of satellite in inertial frame.
            length (float): Norm of position vector.
            attitude (numpy.ndarray): Attitude of satellite.

        Returns:
            numpy.ndarray : gravity acceleration vector
        '''
        coeff  = self.MU / length**2
        if self.hi_fi:
            accel = self.hi_fi_gravity(position, length, coeff)
        else:
            accel = -coeff * position / length
        return accel

    def magnetic_field(self, r_ecef, length, GCI_to_ECEF):
        '''Magnetic field dipole model, average 20-50 uT magnitude. Gradient of 1st order term of IGRF model of magnetic potential.
        See page 403 - 406 or https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details.
        Be aware that every year, the approximated coefficients change a little.

        Parameters:
            r_ecef (numpy.ndarray): Position of satellite in Earth-centered Earth-fixed frame.
            length (float): Norm of position vector.
            GCI_to_ECEF (numpy.ndarray): Matrix for coordinate transformation from inertial frame to ECEF frame.

        Returns: 
            numpy.ndarray: Magnetic B-field (T) in inertial coordinates.
        '''
        R      = (3 * np.dot(self.m, r_ecef) * r_ecef - self.m * length**2) / length**5 # nT
        B      = GCI_to_ECEF.T.dot(R)
        return B



class OrbitalEnvironment():
    AU    = 1.496e11 # m, 1 mean distance from earth to sun
    EARTH_ROTATION = 7.2921158553e-5 #: Rotation of Earth around its axis (rad/s).
    G_NEWTON = 6.6743e-11 #m^3 kg^-1 s^-2
    EARTH_RAD = 6.378137e6 # m, earth mean equitorial radius
    EARTH_MASS = 5.9733328e24 # kg, mass of earth including atmosphere
    
    # ref markely & crassidis
    J2 = 1.08262668355e-3
    J3 = -2.53265648533e-6
    J4 = -1.61962159137e-6
    
    # for geocentric magnetic field model
    # see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
    MAG_REF_RADIUS = 6.3712e6 # m, magnetic spherical ref. radius
    # these coefficients are in nT. from IGRF-13, 2020.0
    G_1_1 = -1450.9 # secular variation 7.4 / year
    H_1_1 = 4652.5 # secular variation -25.9 / year
    G_1_0 = -29404.8 # secular variation 5.7 / year

    '''
    Simplified environmental models for sun, aerodynamics, and gravity.
    '''
    def __init__(self, hi_fi=False):
        self.hi_fi = hi_fi
        # Until I calcualte these, just keep them here
        self.MU = self.G_NEWTON * self.EARTH_MASS #: Gravitational parameter of satellite w.r.t. Earth.
        self.m     = self.MAG_REF_RADIUS**3 * 1E-09 * np.array([self.G_1_1, self.H_1_1, self.G_1_0]) # magnetic dipole in ECEF
        self.SRP_coeff = 1362 * self.AU**2 / 299792458 # Newtons, eq D.56


    def relative_vel(self, orbit):
        '''Relative velocity of satellite, with respect to the atmosphere at this location,
        assuming atmosphere rotates with earth with respect to inertial frame.

        Parameters:
            x (numpy.ndarray): Position of satellite in inertial frame.
            v (numpy.ndarray): Velocity of satellite in inertial frame.

        Returns:
            numpy.ndarray : Relative velocity of satellite.
        '''
        v_rel = np.array([orbit.v[0] + self.EARTH_ROTATION * orbit.x[1],
                          orbit.v[1] + self.EARTH_ROTATION * orbit.x[0],
                          orbit.v[2]])
        return v_rel # m/s


    def SRP_info(self, orbit):
        '''Unit vector in inertial coordinates pointing from satellite to the sun.
        More details on page 420 of Markely & Crassidis. For now assume the sun is a constant distance away.

        Parameters:
            orbit: structure with position in inertial frame and clock

        Returns:
            float: Solar radiation pressure (N/m^2)
            bool: True if position is in earth shadow
            numpy.ndarray: normalized inertial unit vector pointing at the sun from the satellite
        '''
        T_UT1      = (orbit.clock.julian_date() - 2451545) / 36525
        mean_long  = (280.46 + 36000.771 * T_UT1) % 360 # degrees mean longitude
        mean_anom  = np.radians((357.5277233 + 35999.05034 * T_UT1) % 360) # rad mean anomaly
        ecl_long   = np.radians(mean_long + 1.914666471 * np.sin(mean_anom) + 0.019994643 * np.sin(2*mean_anom)) # ecliptic longitude
        ecl_oblq   = np.radians(23.439291 - 0.0130042 * T_UT1)
        
        earth_to_sun = np.array([np.cos(ecl_long),
                                np.cos(ecl_oblq) * np.sin(ecl_long),
                                np.sin(ecl_oblq) * np.sin(ecl_long)])

        sat_to_sun   = self.AU * earth_to_sun - orbit.x

        S_inertial = vector.normalize(sat_to_sun)
        in_shadow    = np.dot(orbit.x, earth_to_sun) < - np.sqrt(np.dot(orbit.x, orbit.x) - self.EARTH_RAD**2) # cylindrical approx for earth shadowing

        SRP          = self.SRP_coeff / np.dot(sat_to_sun, sat_to_sun) # solar radiation pressure
        return SRP, in_shadow, S_inertial 



    def atmo_density(self, orbit):
        '''Exponentially decaying atmosphere model. Refer to page 406 of Markely & Crassidis.
        Eventually we may want a higher fidelity model.
        Be aware that this is undefined outside of 250 - 500 km.

        Parameters:
            h (float): Height (m) above geode in geodetic coordinates.

        Returns:
            float: Atmospheric density (kg/m^3).
        '''
        if 250000 <= orbit.h < 300000:
            h_0   = 250000 # m
            rho_0 = 7.248 * 10**(-11) # kg/m^3
            H     = 46900 # m
        elif 300000 <= orbit.h < 350000:
            h_0   = 300000 # m
            rho_0 = 2.418 * 10**(-11) # kg/m^3
            H     = 52500 # m
        elif 350000 <= orbit.h < 400000:
            h_0   = 350000 # m
            rho_0 = 9.158 * 10**(-12) # kg/m^3
            H     = 56400 # m
        elif 400000 <= orbit.h < 450000:
            h_0   = 400000 # m
            rho_0 = 3.727 * 10**(-12) # kg/m^3
            H     = 59400 # m
        elif 450000 <= orbit.h < 500000:
            h_0   = 400000 # m
            rho_0 = 1.585 * 10**(-12) # kg/m^3
            H     = 62200 # m
        else:
            print("height out of bounds!", orbit.h) # when less lazy, allow for decaying orbit
        return rho_0 * np.exp((h_0 - orbit.h) / H) # kg/m^3


    def hi_fi_gravity(self, orbit, coeff):
        '''Higher order gravity model including J2, J3, and J4 zonal terms.
        Refer to Markely and Crassidis.

        Parameters:
            position (numpy.ndarray): Position of satellite in inertial frame.
            r (float): Norm of position vector.
            coeff (float) For computational convenience, mu / r^2.

        Returns:
            numpy.ndarray: Gravitational acceleration.
        '''
        xoverr = orbit.position[0] / orbit.length
        yoverr = orbit.position[1] / orbit.length
        zoverr = orbit.position[2] / orbit.length
        zoverrsquare = zoverr**2
        zoverrcube = zoverr*zoverrsquare
        zoverr4th = zoverrsquare**2
        rearthoverr = self.EARTH_RAD / orbit.length
        a   = orbit.position / orbit.length
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


    def gravity_accel(self, orbit):
        '''Gravitational forces and torques.
        Note that gravity torque is fairly predictable and we could even use it for feedforward in controls.

        Parameters: 
            position (numpy.ndarray): Position of satellite in inertial frame.
            length (float): Norm of position vector.
            attitude (numpy.ndarray): Attitude of satellite.

        Returns:
            numpy.ndarray : gravity acceleration vector
        '''
        coeff  = self.MU / orbit.length**2
        if self.hi_fi:
            accel = self.hi_fi_gravity(orbit, coeff)
        else:
            accel = -coeff * orbit.position / orbit.length
        return accel

    def magnetic_field(self, orbit):
        '''Magnetic field dipole model, average 20-50 uT magnitude. Gradient of 1st order term of IGRF model of magnetic potential.
        See page 403 - 406 or https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details.
        Be aware that every year, the approximated coefficients change a little.

        Parameters:
            r_ecef (numpy.ndarray): Position of satellite in Earth-centered Earth-fixed frame.
            length (float): Norm of position vector.
            GCI_to_ECEF (numpy.ndarray): Matrix for coordinate transformation from inertial frame to ECEF frame.

        Returns: 
            numpy.ndarray: Magnetic B-field (T) in inertial coordinates.
        '''
        R      = (3 * np.dot(self.m, orbit.r_ecef) * orbit.r_ecef - self.m * orbit.length**2) / orbit.length**5 # nT
        B      = orbit.GCI_to_ECEF.T.dot(R)
        return B

