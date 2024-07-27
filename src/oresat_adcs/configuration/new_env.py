import numpy as np
from ..functions import frame, quaternion, vector



class Environment():
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
        return a   # relative_vel exists in parent class




    def hi_fi_gravity(self, position):
        '''
        Higher order gravity model including J2, J3, and J4 zonal terms.
        Refer to Markely and Crassidis.

        Parameters
        ----------
        position : numpy.ndarray
            Position of satellite in inertial frame.

        Returns
        -------
        numpy.ndarray
            Gravitational acceleration.
        '''
        length = np.linalg.norm(position)
        coeff = self.MU / length**2
        r = length

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





    def magnetic_field(self, x, GCI_to_ECEF):
        '''
        Magnetic field dipole model, average 20-50 uT magnitude. Gradient of 1st order term of IGRF model of magnetic potential.
        See page 403 - 406 or https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details.
        Be aware that every year, the approximated coefficients change a little.


        Parameters
        ----------
        r_ecef : numpy.ndarray
            Position of satellite in Earth-centered Earth-fixed frame.
        length : float
            Norm of position vector.
        GCI_to_ECEF : numpy.ndarray
            Matrix for coordinate transformation from inertial frame to ECEF frame.

        Returns
        -------
        numpy.ndarray
            Magnetic B-field (T) in inertial coordinates.
        '''
        r_ecef = GCI_to_ECEF.dot(x)
        length = np.linalg.norm(x)
        R      = (3 * np.dot(self.m, r_ecef) * r_ecef - self.m * length**2) / length**5 # nT
        B      = GCI_to_ECEF.T.dot(R)
        return B


