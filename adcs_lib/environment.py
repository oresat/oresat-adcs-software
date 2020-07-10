import numpy as np
import frame, quaternion

class Environment():
    '''Environmental models.'''

    def __init__(self, satellite):
        self.EARTH_ROTATION = 7.2921158553e-5 #: Rotation of Earth around its axis (rad/s).
        self.G_NEWTON = 6.6743e-11 #m^3 kg^-1 s^-2
        self.EARTH_MASS = 5.972e24 # kg
        #EARTH_MU = 3.986004418e14 # m^3 s^-2, standard gravitational parameter of earth
        self.MU = G_NEWTON * (satellite.mass + EARTH_MASS) #: Gravitational parameter of satellite w.r.t. Earth.

        # for geocentric magnetic field model
        # see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
        self.MAG_REF_RADIUS = 6.3712e6 # m, magnetic spherical ref. radius
        # these coefficients are in nT. from IGRF-13, 2020.0
        self.G_1_1 = -1450.9 # secular variation 7.4 / year
        self.H_1_1 = 4652.5 # secular variation -25.9 / year
        self.G_1_0 = -29404.8 # secular variation 5.7 / year
        self.m     = self.MAG_REF_RADIUS**3 * 1E-09 * np.array([self.G_1_1, self.H_1_1, self.G_1_0]) # magnetic dipole in ECEF
        self.satellite = satellite

    # exponentially decaying model atmosphere page 406
    # h is height above geode in m (i.e., in geodetic coordinates)
    def atmo_density(self, h):
        if 350000 <= h and h < 400000:
            h_0   = 350000 # m
            rho_0 = 9.158 * 10**(-12) # kg/m^3
            H     = 56400 # m
        elif 400000 <= h and h < 450000:
            h_0   = 400000 # m
            rho_0 = 3.727 * 10**(-12) # kg/m^3
            H     = 59400 # m
        else:
            print("height out of bounds!", h) # when less lazy, allow for decaying orbit
        return rho_0 * np.exp((h_0 - h) / H) # kg/m^3

    # assume atmosphere rotates with earth
    # relative velocity of satellite with respect to atmosphere at location on earth
    # with respect to inertial frame and expressed in body frame
    def relative_vel(self, x, v):
        v_rel = np.array([v[0] + self.EARTH_ROTATION * x[1],
                          v[1] + self.EARTH_ROTATION * x[0],
                          v[2]])
        return v_rel # m/s

    # drag equation, unit agnostic
    def drag(self, rho, v, attitude):
        v_norm   = np.linalg.norm(v)
        v_ref    = quaternion.sandwich(attitude, v / v_norm)
        (A, CoP) = self.satellite.area_and_cop(v_ref)
        F        = -0.5 * rho * self.satellite.drag_coeff * v_norm * v * A
        T        = np.cross(CoP, quaternion.sandwich(attitude, F))
        return np.array([F, T])

    def gravity(self, position, length, attitude):
        coeff  = MU / length**3
        F      = -coeff * position * self.satellite.mass
        n      = quaternion.sandwich(attitude, -position / length)
        T      = 3 * coeff * np.cross(n, self.satellite.total_moment.dot(n))
        return np.array([F, T])


    # dipole model
    # gradient of 1st order term of IGRF model of magnetic potential
    # see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
    # be aware that every year, these approximated coefficients change a little
    # see page 403 - 406 for details. i expect 20-50 uT magnitude
    def magnetic_field(self, r_ecef, length, GCI_to_ECEF):
        R      = (3 * np.dot(self.m, r_ecef) * r_ecef - self.m * length**2) / length**5 # nT
        B      = GCI_to_ECEF.T.dot(R)
        return B

    def env_F_and_T(self, position, velocity, attitude, GCI_to_ECEF, mag_moment):
        r_ecef       = GCI_to_ECEF.dot(position)
        length       = np.linalg.norm(position)
        B            = self.magnetic_field(r_ecef, length, GCI_to_ECEF)
        B_body       = quaternion.sandwich(attitude, B)
        lat, long, h = frame.ECEF_to_geodetic(r_ecef)
        rho          = self.atmo_density(h)
        v_rel        = self.relative_vel(postion, velocity)

        D = self.drag(rho, v_rel, attitude)
        G = self.gravity(position, length, attitude)
        M = np.array([np.zeros(3), np.cross(mag_moment, B_body)])
        return D + G + M
