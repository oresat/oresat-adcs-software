import numpy as np
from ..functions import frame, quaternion, vector

class ReducedEnvironment():
    '''Simplified environmental models for sun, aerodynamics, and gravity.
    Parameters
    ----------
    config : dict
        A dictionary of dictionaries, keys should be variable names, 
        dictionaries should be {"value": int, "desc": str}
        Example: "CD" : {"value": 2, "desc": "coefficient of drag"}
    '''
    def __init__(self, satellite, config):
        # for geocentric magnetic field model
        # see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
        
        required = ["AU", "EARTH_ROTATION", "G_NEWTON", "EARTH_RAD", "EARTH_MASS",
                    "J2", "J3", "J4", "RHO", "CD", "A", "M", "MAG_REF_RADIUS",
                    "G_1_1", "H_1_1", "G_1_0"]
        
        for attribute in required:
            if attribute not in config.keys():
                print("Error, required value not found")
            self.__dict__[attribute] = config[attribute]["value"]

        self.MU = self.G_NEWTON * self.EARTH_MASS #: Gravitational parameter of satellite w.r.t. Earth.
        self.m     = self.MAG_REF_RADIUS**3 * 1E-09 * np.array([self.G_1_1, self.H_1_1, self.G_1_0]) # magnetic dipole in ECEF
        
        self.satellite = satellite

    def earth_sun_vector(self, clock):
        '''Intermediate function for calculating the earth to sun fector

        Parameters
        ----------
        clock : jday.Clock
            Clock is needed because the position of the sun depends on the date and time (obviously).
        x : numpy.ndarray
            Position of the satellite in inertial frame.

        Returns
        -------
        numpy.ndarray
            Earth to sun vector
        '''
        T_UT1      = (clock.julian_date(clock.hour, clock.minute, clock.second) - 2451545) / 36525
        mean_long  = (280.46 + 36000.771 * T_UT1) % 360 # degrees mean longitude
        mean_anom  = np.radians((357.5277233 + 35999.05034 * T_UT1) % 360) # rad mean anomaly
        ecl_long   = np.radians(mean_long + 1.914666471 * np.sin(mean_anom) + 0.019994643 * np.sin(2*mean_anom)) # ecliptic longitude
        ecl_oblq   = np.radians(23.439291 - 0.0130042 * T_UT1)
        earth_to_sun = np.array([np.cos(ecl_long), np.cos(ecl_oblq) * np.sin(ecl_long), np.sin(ecl_oblq) * np.sin(ecl_long)])
        return earth_to_sun


    def sun_vector(self, clock, x):
        '''Unit vector in inertial coordinates pointing from satellite to the sun.
        More details on page 420 of Markely & Crassidis. For now assume the sun is a constant distance away.

        Parameters
        ----------
        clock : jday.Clock
            Clock is needed because the position of the sun depends on the date and time (obviously).
        x : numpy.ndarray
            Position of the satellite in inertial frame.

        Returns
        -------
        numpy.ndarray
            Inertial unit vector pointing at sun from satellite.
        '''
        T_UT1      = (clock.julian_date(clock.hour, clock.minute, clock.second) - 2451545) / 36525
        mean_long  = (280.46 + 36000.771 * T_UT1) % 360 # degrees mean longitude
        mean_anom  = np.radians((357.5277233 + 35999.05034 * T_UT1) % 360) # rad mean anomaly
        ecl_long   = np.radians(mean_long + 1.914666471 * np.sin(mean_anom) + 0.019994643 * np.sin(2*mean_anom)) # ecliptic longitude
        ecl_oblq   = np.radians(23.439291 - 0.0130042 * T_UT1)
        earth_to_sun = np.array([np.cos(ecl_long),
                                np.cos(ecl_oblq) * np.sin(ecl_long),
                                np.sin(ecl_oblq) * np.sin(ecl_long)])

        # Remove stuff above this line
        
        earth_to_sun = self.earth_sun_vector(clock)
        S_inertial = vector.pointing_vector(x, self.AU * earth_to_sun)
        return S_inertial

    def relative_vel(self, x, v):
        '''
        Relative velocity of satellite, with respect to the atmosphere at this location,
        assuming atmosphere rotates with earth with respect to inertial frame.

        Parameters
        ----------
        x : numpy.ndarray
            Position of satellite in inertial frame.
        v : numpy.ndarray
            Velocity of satellite in inertial frame.

        Returns
        -------
        numpy.ndarray
            Relative velocity of satellite.
        '''
        v_rel = np.array([v[0] + self.EARTH_ROTATION * x[1],
                          v[1] + self.EARTH_ROTATION * x[0],
                          v[2]])
        return v_rel # m/s

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

    def grav_torque(self, position, attitude, MoI):
        length = np.linalg.norm(position)
        coeff  = self.MU / length**2
        n      = quaternion.sandwich(attitude, -position / length)
        T      = 3 * coeff / length * np.cross(n, MoI.dot(n))
        return T

    def forces(self, x, v):
        '''
        Forces acting on satellite.

        Parameters
        ----------
        x : numpy.ndarray
            Position of satellite in inertial frame.
        v : numpy.ndarray
            Velocity of satellite in inertial frame.

        Returns
        -------
        numpy.ndarray
            Total forces acting on satellite.
        '''
        v_rel = self.relative_vel(x, v)
        D = self.drag(v_rel)
        G = self.gravity(x) * self.M
        return D + G

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

