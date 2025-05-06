import numpy as np
from ..functions import frame, quaternion, vector
from scipy.special import lpmv


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
        
        # self.g_coeffs and self.h_coeffs are from an attempt at 2nd degree IGRF implementation
        # IGRF-13  2020 coefficients (nT)
       
        self.a = 6371.2e3 

        self.g_coeffs = {
            (1, 0): -29404.8,
            (1, 1): -1450.9,
            (2, 0): -2499.6,
            (2, 1): 2982.0,
            (2, 2): 1677.0,
        }

        self.h_coeffs = {
            (1, 1): 4652.5,
            (2, 1): -2991.6,
            (2, 2): -734.6,
        }
    # the lines below were for the 3rd degree attempt
        self.g_coeffs.update({
            (3, 0): 1363.2,
            (3, 1): -2381.2,
            (3, 2): 1236.2,
            (3, 3): 525.7
       })
        self.h_coeffs.update({
            (3, 1): -82.1,
            (3, 2): 241.9 ,
            (3, 3): -543.4
       })

    def relative_vel(self, state):
        '''Relative velocity of satellite, with respect to the atmosphere at this location,

        Parameters:
            x (numpy.ndarray): Position of satellite in inertial frame.
            v (numpy.ndarray): Velocity of satellite in inertial frame.

        Returns:
            numpy.ndarray : Relative velocity of satellite.
        '''
        v_rel = np.array([state.v[0] + self.EARTH_ROTATION * state.x[1],
                          state.v[1] + self.EARTH_ROTATION * state.x[0],
                          state.v[2]])
        return v_rel # m/s


    def SRP_info(self, state):
        '''Unit vector in inertial coordinates pointing from satellite to the sun.
        More details on page 420 of Markely & Crassidis. For now assume the sun is a constant distance away.

        Parameters:
            orbit: structure with position in inertial frame and clock

        Returns:
            float: Solar radiation pressure (N/m^2)
            bool: True if position is in earth shadow
            numpy.ndarray: normalized inertial unit vector pointing at the sun from the satellite
        '''
        T_UT1      = (state.clock.julian_date() - 2451545) / 36525
        mean_long  = (280.46 + 36000.771 * T_UT1) % 360 # degrees mean longitude
        mean_anom  = np.radians((357.5277233 + 35999.05034 * T_UT1) % 360) # rad mean anomaly
        ecl_long   = np.radians(mean_long + 1.914666471 * np.sin(mean_anom) + 0.019994643 * np.sin(2*mean_anom)) # ecliptic longitude
        ecl_oblq   = np.radians(23.439291 - 0.0130042 * T_UT1)
        
        earth_to_sun = np.array([np.cos(ecl_long),
                                np.cos(ecl_oblq) * np.sin(ecl_long),
                                np.sin(ecl_oblq) * np.sin(ecl_long)])

        sat_to_sun   = self.AU * earth_to_sun - state.x

        S_inertial = vector.normalize(sat_to_sun)

        in_shadow    = np.dot(state.x, earth_to_sun) < - np.sqrt(np.dot(state.x, state.x) - self.EARTH_RAD**2) # cylindrical approx for earth shadowing

        SRP          = self.SRP_coeff / np.dot(sat_to_sun, sat_to_sun) # solar radiation pressure
        return SRP, in_shadow, S_inertial 



    def atmo_density(self, state):
        '''Exponentially decaying atmosphere model. Refer to page 406 of Markely & Crassidis.
        Eventually we may want a higher fidelity model.
        Be aware that this is undefined outside of 250 - 500 km.

        Parameters:
            h (float): Height (m) above geode in geodetic coordinates.

        Returns:
            float: Atmospheric density (kg/m^3).
        '''
        if 250000 <= state.h < 300000:
            h_0   = 250000 # m
            rho_0 = 7.248 * 10**(-11) # kg/m^3
            H     = 46900 # m
        elif 300000 <= state.h < 350000:
            h_0   = 300000 # m
            rho_0 = 2.418 * 10**(-11) # kg/m^3
            H     = 52500 # m
        elif 350000 <= state.h < 400000:
            h_0   = 350000 # m
            rho_0 = 9.158 * 10**(-12) # kg/m^3
            H     = 56400 # m
        elif 400000 <= state.h < 450000:
            h_0   = 400000 # m
            rho_0 = 3.727 * 10**(-12) # kg/m^3
            H     = 59400 # m
        elif 450000 <= state.h < 500000:
            h_0   = 400000 # m
            rho_0 = 1.585 * 10**(-12) # kg/m^3
            H     = 62200 # m
        else:
            print("height out of bounds!", state.h) # when less lazy, allow for decaying orbit
        return rho_0 * np.exp((h_0 - state.h) / H) # kg/m^3


    def hi_fi_gravity(self, state, coeff):
        '''Higher order gravity model including J2, J3, and J4 zonal terms.
        Refer to Markely and Crassidis.

        Parameters:
            position (numpy.ndarray): Position of satellite in inertial frame.
            r (float): Norm of position vector.
            coeff (float) For computational convenience, mu / r^2.

        Returns:
            numpy.ndarray: Gravitational acceleration.
        '''
        xoverr = state.position[0] / state.length
        yoverr = state.position[1] / state.length
        zoverr = state.position[2] / state.length
        zoverrsquare = zoverr**2
        zoverrcube = zoverr*zoverrsquare
        zoverr4th = zoverrsquare**2
        rearthoverr = self.EARTH_RAD / state.length
        a   = state.position / state.length
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


    def gravity_accel(self, state):
        '''Gravitational forces and torques.
        Note that gravity torque is fairly predictable and we could even use it for feedforward in controls.

        Parameters: 
            position (numpy.ndarray): Position of satellite in inertial frame.
            length (float): Norm of position vector.
            attitude (numpy.ndarray): Attitude of satellite.

        Returns:
            numpy.ndarray : gravity acceleration vector
        '''
        coeff  = self.MU / state.length**2
        if self.hi_fi:
            accel = self.hi_fi_gravity(state, coeff)
        else:
            accel = -coeff * state.position / state.length
        return accel

    def magnetic_field(self, state):
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
        R      = (3 * np.dot(self.m, state.r_ecef) * state.r_ecef - self.m * state.length**2) / state.length**5 # nT
        B      = state.GCI_to_ECEF.T.dot(R)
        return B

    def compute_dPnm(self, m, n, cos_theta, theta):
        """
        Compute derivative of associated Legendre function w.r.t theta.
        """
        Pnm = lpmv(m, n, cos_theta)
        if abs(np.sin(theta)) < 1e-10:
            return 0.0  # avoid divide by zero at poles

        return (n * np.cos(theta) * Pnm - (n + m) * lpmv(m, n - 1, cos_theta)) / np.sin(theta)    

    def magnetic_field_inertial(self,state):
        """
        Compute magnetic field up to degree 2 in inertial frame.
        r_inertial: position vector (m), inertial frame
        Returns: magnetic field vector (T), inertial frame
       """
        r_inertial = state.position 

        r_vec = np.array( r_inertial)
        r = np.linalg.norm(r_vec)
        theta = np.arccos(r_vec[2] / r)  # colatitude (rad)
        phi = np.arctan2(r_vec[1], r_vec[0])  # longitude (rad)

        # Initialize spherical components
        Br = 0.0
        Btheta = 0.0
        Bphi = 0.0
        
        # Loop over degrees n = 1, 2
        for n in [1, 2]:
            for m in range(0, n + 1):
                g = self.g_coeffs.get((n, m), 0.0)
                h = self.h_coeffs.get((n, m), 0.0)

                Pnm = lpmv(m, n, np.cos(theta))  # Associated Legendre
                dPnm = self.compute_dPnm(m, n, np.cos(theta), theta)  # YOU NEED TO DEFINE THIS DERIVATIVE

                factor = (self.a / r) ** (n + 2)
                cos_mphi = np.cos(m * phi)
                sin_mphi = np.sin(m * phi)

                Br += factor * (n + 1) * (g * cos_mphi + h * sin_mphi) * Pnm
                Btheta -= factor * (g * cos_mphi + h * sin_mphi) * dPnm
                if m != 0:
                    Bphi -= factor * m * (g * sin_mphi - h * cos_mphi) * Pnm

        # Convert from nT to Tesla
        Br *= 1e-9
        Btheta *= 1e-9
        Bphi *= 1e-9

        # Convert spherical (Br, Btheta, Bphi) to Cartesian (Bx, By, Bz)
        Bx = (np.sin(theta) * np.cos(phi) * Br +
              np.cos(theta) * np.cos(phi) * Btheta -
              np.sin(phi) * Bphi)
        By = (np.sin(theta) * np.sin(phi) * Br +
              np.cos(theta) * np.sin(phi) * Btheta +
              np.cos(phi) * Bphi)
        Bz = (np.cos(theta) * Br - np.sin(theta) * Btheta)

        return np.array([Bx, By, Bz])


    def magnetic_field_inertial_deg3(self,state):
        """
        Compute magnetic field up to degree 2 in inertial frame.
        r_inertial: position vector (m), inertial frame
        Returns: magnetic field vector (T), inertial frame
       """
        r_inertial = state.position 

        r_vec = np.array( r_inertial)
        r = np.linalg.norm(r_vec)
        theta = np.arccos(r_vec[2] / r)  # colatitude (rad)
        phi = np.arctan2(r_vec[1], r_vec[0])  # longitude (rad)

        # Initialize spherical components
        Br = 0.0
        Btheta = 0.0
        Bphi = 0.0

        # Loop over degrees n = 1, 2
        for n in [3]:
            for m in range(0, n + 1):
                g = self.g_coeffs.get((n, m), 0.0)
                h = self.h_coeffs.get((n, m), 0.0)

                Pnm = lpmv(m, n, np.cos(theta))  # Associated Legendre
                dPnm = self.compute_dPnm(m, n, np.cos(theta), theta)  # YOU NEED TO DEFINE THIS DERIVATIVE

                factor = (self.a / r) ** (n + 2)
                cos_mphi = np.cos(m * phi)
                sin_mphi = np.sin(m * phi)

                Br += factor * (n + 1) * (g * cos_mphi + h * sin_mphi) * Pnm
                Btheta -= factor * (g * cos_mphi + h * sin_mphi) * dPnm
                if m != 0:
                    Bphi -= factor * m * (g * sin_mphi - h * cos_mphi) * Pnm

        # Convert from nT to Tesla
        Br *= 1e-9
        Btheta *= 1e-9
        Bphi *= 1e-9

        # Convert spherical (Br, Btheta, Bphi) to Cartesian (Bx, By, Bz)
        Bx = (np.sin(theta) * np.cos(phi) * Br +
              np.cos(theta) * np.cos(phi) * Btheta -
              np.sin(phi) * Bphi)
        By = (np.sin(theta) * np.sin(phi) * Br +
              np.cos(theta) * np.sin(phi) * Btheta +
              np.cos(phi) * Bphi)
        Bz = (np.cos(theta) * Br - np.sin(theta) * Btheta)

        return np.array([Bx, By, Bz])

    def magnetic_field_inertial_up_to_n(self, state, max_degree):
        r_vec = np.array(state.position)
        r = np.linalg.norm(r_vec)
        theta = np.arccos(r_vec[2] / r)
        phi = np.arctan2(r_vec[1], r_vec[0])

        # looking at B(the geomagnetic field) at a certain time(so Bt is ommitted)
        Br, Btheta, Bphi = 0.0, 0.0, 0.0

        for n in range(1, max_degree + 1):
            for m in range(0, n + 1):
                # gets constants g and m
                g = self.g_coeffs.get((n, m), 0.0)
                h = self.h_coeffs.get((n, m), 0.0)
                
                # does Legendre functions math for the Pnm(cos(theta)) in V()
                Pnm = lpmv(m, n, np.cos(theta))
                dPnm = self.compute_dPnm(m, n, np.cos(theta), theta)

                # Factor is the a(a/r)^(n+1) in the spherical harmonic expansion V()
                factor = (self.a / r) ** (n + 2)
                 
                # below is that cosmphi and sinmphi cals for V()
                cos_mphi = np.cos(m * phi)
                sin_mphi = np.sin(m * phi)

 
                #  derivative for V() wrt r. this is why we have += (n+1)
                #  rather than -= ___ 
                Br += factor * (n + 1) * (g * cos_mphi + h * sin_mphi) * Pnm

                #  derivative for V() wrt theta. Pnm(cos) is the only factor
                # theta. this is why we use dPnm
                Btheta -= factor * (g * cos_mphi + h * sin_mphi) * dPnm

                # derivative for V() wrt phi. cos and sin are only phi terms, they are                 #  flip flopped to satisfy derivation
                if m != 0:
                    Bphi -= factor * m * (g * sin_mphi - h * cos_mphi) * Pnm

        # convert from tesla to nano tesla
        Br *= 1e-9
        Btheta *= 1e-9
        Bphi *= 1e-9

        # convert from spherical(r, phi, theta into euler coords)
        Bx = (np.sin(theta) * np.cos(phi) * Br +
              np.cos(theta) * np.cos(phi) * Btheta -
              np.sin(phi) * Bphi)
        By = (np.sin(theta) * np.sin(phi) * Br +
             np.cos(theta) * np.sin(phi) * Btheta +
             np.cos(phi) * Bphi)
        Bz = (np.cos(theta) * Br - np.sin(theta) * Btheta)

        return np.array([Bx, By, Bz])
         
