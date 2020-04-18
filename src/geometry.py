import numpy as np

# Reference Fundamentals of Spacecraft Attitude Determination and Control
# by Markely and Crassidis for understanding, pardon the magic numbers
# This is needed to translate between ECEF and ECI coordinates
class Clock():
    def __init__(self, year, month, day, hour, minute, second, dt):
        self.year = year
        self.month = month
        self.day = day
        self.hour = hour
        self.minute = minute
        self.second = second
        self.leap_year = is_leap_year()
        self.leap_second = False

    def schedule_leap_second(self, year, month, day, hour, minute, second):
        pass # figure it out later
    def is_leap_second(self):
        pass # probably just check whether state matches schedule
    def is_leap_year(self):
        if not (self.year % 4 == 0):
            return False
        elif not (self.year % 100 == 0):
            return True
        elif not (self.year % 400 == 0):
            return False
        else:
            return True
    # advance one time step
    def tick(self): # naive implementation, I hate the Gregorian calendar
        self.second += self.dt
        if ((not self.leap_second and self.second + self.dt == 60) or
            (self.leap_second and self.second == 61)):
            self.second = 0
            self.minute += 1

        if self.minute == 60:
            self.minute = 0
            self.hour += 1
        if self.hour == 24:
            self.hour = 0
            self.day += 1

        if ((self.month in [1, 3, 5, 7, 8, 10, 12] and self.day == 32) or
            (self.month in [4, 6, 9, 11] and self.day == 31) or
            (self.month == 2 and ((not self.leap_year and self.day == 29) or
                                    (self.leap_year and self.day == 30)))):
            self.day = 1
            self.month += 1

        if self.month == 13:
            self.month = 1
            self.year += 1
            self.leap_year = self.is_leap_year()
    # we could cache some of this intelligently
    def julian_date(self, hour, minute, second):
        term1 = int(7/4 * (self.year + int((self.month + 9) / 12)))
        term2 = int(275*self.month / 9)
        term3 = (60*hour + minute + second/(60+self.leap_second)) / 1440
        return 1721013.5 + 367*self.year + self.day - term1 + term2 + term3
    # from epoch J2000.0 to zero hour now
    def centuries_elapsed(self):
        return (self.julian_date(0,0,0) - 2451545) / 36525
    # Greenwich Mean Sidereal time
    def gmst_seconds(self):
        centuries = self.centuries_elapsed()
        term1 = 8640184.812866 * centuries + 0.093104 * centuries**2 - 6.2*10**(-6) * centuries**3
        term2 = 1.002737909350795 * (3600 * self.hour + 60 * self.minute + self.second)
        return 24110.54841 + term1 + term2
    # angle between vernal equinox and Greenwich mean line
    def theta_gmst(self):
        gmst = self.gmst_seconds() % 86400
        return np.radians(gmst / 240)


## these next few functions are for our use of quaternions in implementating rotations
# this is the inverse of a unit quat
def conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

# this constrains a quat to S^3
def normalize(q):
    return q / np.linalg.norm(q)

# this is hamilton's quaternion product, q[0] is real part of a quaternion
# be aware that some aerospace textbooks use a different sign convention on cross product
def quat_product(a, b):
    v = b[0] * a[1:] + a[0] * b[1:] + np.cross(a[1:], b[1:])
    return np.array([a[0] * b[0] - np.dot(a[1:], b[1:]), v[0], v[1], v[2]])

# this expresses a vector in the reference frame of the quaterion
def sandwich(q, v):
    return product(conjugate(q), product(np.array([0, v[0], v[1], v[2]]), q))[1:]

# encodes an axis-angle representation of a rotation as a unit quaterion
def axisangle_to_quat(r, theta):
    v = np.sin(theta/2) * r
    return np.array([np.cos(theta/2), v[0], v[1], v[2]])

# encodes startracker attitude representation as quaterion
# 3-2-1 order. transforms inertial frame to body reference frame
def eulerangle_to_quat(RA, dec, orientation):
    RA = np.radians(RA) / 2
    dec = np.radians(dec) / 2
    ortn = np.radians(orientation) / 2
    c_phi = np.cos(RA)
    s_phi = np.sin(RA)
    c_theta = np.cos(dec)
    s_theta = np.sin(dec)
    c_psi = np.cos(ortn)
    s_psi = np.sin(ortn)
    return np.array([c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
                     c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
                     c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
                     s_phi * c_theta * c_psi + c_phi * s_theta * s_psi])


## these next few functions are for transformations between coordinate systems
# transformation from inertial frame to ECEF, Cartesian x, y, z
def inertial_to_ecef(clock):
    gmst = clock.theta_gmst()
    C = np.cos(gmst)
    S = np.sin(gmst)
    return np.array([[C, S, 0],
                     [-S, C, 0],
                     [0, 0, 1]])

# transformation from local orbital frame to inertial frame, Cartesian x, y, z
def lvlh_to_inertial(r_I, v_I):
    o_3I = - r_I / np.linalg.norm(r_I)
    o_2I = - np.cross(r_I, v_I) / np.linalg.norm(np.cross(r_I, v_I))
    o_1I = np.cross(o_2I, o_3I)
    return np.array([[o_1I[0], o_2I[0], o_3I[0]],
                     [o_1I[1], o_2I[1], o_3I[1]],
                     [o_1I[2], o_2I[2], o_3I[2]]])

# transformation from WGS-84 geodetic coordinates to ECEF geocentric coordinates
# might not need this if GPS is already in ECEF
def geodetic_to_ECEF(lat, long, h):
    a = 6378137.0 # m, semimajor axis
    e = 0.0818 # eccentricity approximation
    N = a / np.sqrt(1 - (e*np.sin(lat))**2)
    temp = (N + h) * np.cos(lat)
    x = temp * np.cos(long)
    y = temp * np.sin(long)
    z = (N*(1 - e**2) + h) * np.sin(lat)
    return np.array([x, y, z])

# transformation from ECEF geocentric coordinates to WGS-84 geodetic coordinates
# might be useful for modeling atmospheric drag
def ECEF_to_geodetic(r):
     a = 6378137.0 # m, semimajor axis
     b = 6356752.3142 # m, semiminor axis
     x = r[0]
     y = r[1]
     z = r[2]
     # convert from ECEF to geodetic coords
     # see page 35 and pray there are no typos in the book
     def geo_rho(x, y):
         return np.linalg.norm(np.array([x, y]))

     def geo_e(a, b):
         return np.sqrt(1 - (b/a)**2)

     def geo_eps(a, b):
         return np.sqrt((a/b)**2 - 1)

     def geo_p(z, eps):
         return abs(z) / eps**2

     def geo_s(rho, e, eps):
         return (rho / (e*eps))**2

     def geo_q(p, b, s):
         return p**2 - b**2 + s

     def geo_u(p, q):
         return p / np.sqrt(q)

     def geo_v(b, u, q):
         return (b*u)**2 / q

     def geo_P(v, s, q):
         return 27*v*s/q

     def geo_Q(P):
         return (np.sqrt(P+1) + np.sqrt(P))**(2/3)

     def geo_t(Q):
         return (1 + Q + 1/Q)/6

     def geo_c(u, t):
         return np.sqrt(u**2 - 1 + 2*t)

     def geo_w(c, u):
         return (c - u)/2

     def geo_d(z, q, u, v, w, t):
         return np.sign(z)*np.sqrt(q)*(w +
                                      np.sqrt(
                                      np.sqrt(t**2 + v) -
                                      u * w - t/2 - 1/4))

     def geo_N(a, eps, d, b):
         return a * np.sqrt(1+ (eps*d/b)**2)

     # latitude
     def geo_lam(eps, d, N):
         return np.arcsin((eps**2 + 1)*d/N)

     # longitude
     def geo_h(rho, z, a, N, lam): # height above geode
         return rho*np.cos(lam) + z*np.sin(lam) - a**2 / N

     # height
     def geo_phi(x, y):
         return np.arctan2(y, x)

     e = geo_e(a, b)
     eps = geo_eps(a, b)
     rho = geo_rho(x, y)
     p = geo_p(z, eps)
     s = geo_s(rho, e, eps)
     q = geo_q(p, b, s)
     u = geo_u(p, q)
     v = geo_v(b, u, q)
     P = geo_P(v, s, q)
     Q = geo_Q(P)
     t = geo_t(Q)
     c = geo_c(u, t)
     w = geo_w(c, u)
     d = geo_d(z, q, u, v, w, t)
     N = geo_N(a, eps, d, b)

     lam = geo_lam(eps, d, N)
     h = geo_h(rho, z, a, N, lam)
     phi = geo_phi(x, y)
     return np.array([lam, phi, h]) # lat, long, height
