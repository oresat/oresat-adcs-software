import numpy as np
from adcs_lib import quaternion

## these next few functions are for transformations between coordinate systems
# transformation from inertial frame to ECEF, Cartesian x, y, z
def inertial_to_ecef(clock):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    gmst = clock.theta_gmst()
    C = np.cos(gmst)
    S = np.sin(gmst)
    return np.array([[C, S, 0],
                     [-S, C, 0],
                     [0, 0, 1]])

# transformation from local orbital frame to inertial frame, Cartesian x, y, z
def lvlh_to_inertial(r_I, v_I):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    o_3I = - quaternion.normalize(r_I)
    o_2I = - quaternion.normalize(np.cross(r_I, v_I))
    o_1I = np.cross(o_2I, o_3I)
    return np.array([[o_1I[0], o_2I[0], o_3I[0]],
                     [o_1I[1], o_2I[1], o_3I[1]],
                     [o_1I[2], o_2I[2], o_3I[2]]])

# transformation from WGS-84 geodetic coordinates to ECEF geocentric coordinates
# might not need this if GPS is already in ECEF
def geodetic_to_ECEF(lat, long, h):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
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

     def geo_e_squared(a, b):
         return 1 - (b**2/a**2)

     def geo_eps_squared(a, b):
         return (a**2/b**2) - 1

     def geo_p(z, eps2):
         return abs(z) / eps2

     def geo_s(rho, e2, eps2):
         return rho**2 / (e2*eps2)

     def geo_q(p, b, s):
         return p**2 - b**2 + s

     def geo_u(p, q):
         return p / np.sqrt(q)

     def geo_v(b, u, q):
         return (b**2 * u**2) / q

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
         return np.sign(z)*np.sqrt(q)*(w + np.sqrt(np.sqrt(t**2 + v) - u * w - t*0.5 - 0.25))

     def geo_N(a, eps2, d, b):
         return a * np.sqrt(1+ eps2*(d**2/b**2))

     # latitude
     def geo_lam(eps2, d, N):
         return np.arcsin((eps2 + 1)*d/N)

     # height
     def geo_h(rho, z, a, N, lam): # height above geode
         return rho*np.cos(lam) + z*np.sin(lam) - a**2 / N

     # longitude
     def geo_phi(x, y):
         return np.arctan2(y, x)

     e2 = geo_e_squared(a, b)
     eps2 = geo_eps_squared(a, b)
     rho = geo_rho(x, y)
     p = geo_p(z, eps2)
     s = geo_s(rho, e2, eps2)
     q = geo_q(p, b, s)
     u = geo_u(p, q)
     v = geo_v(b, u, q)
     P = geo_P(v, s, q)
     Q = geo_Q(P)
     t = geo_t(Q)
     c = geo_c(u, t)
     w = geo_w(c, u)
     d = geo_d(z, q, u, v, w, t)
     N = geo_N(a, eps2, d, b)

     lam = geo_lam(eps2, d, N)
     h = geo_h(rho, z, a, N, lam)
     phi = geo_phi(x, y)
     return np.array([lam, phi, h]) # lat, long, height

import math

# WGS-84 ellipsoid parameters
a = 6378137
f = 1 / 298.257223563

# Derived parameters
e2 = f * (2 - f)
a1 = a * e2
a2 = a1 * a1
a3 = a1 * e2 / 2
a4 = 2.5 * a2
a5 = a1 + a3
a6 = 1 - e2

# source: https://possiblywrong.wordpress.com/2014/02/14/when-approximate-is-better-than-exact/
def ecef_to_lla(ecef):
    """Convert ECEF (meters) to LLA (radians and meters).
    """
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    # Olson, D. K., Converting Earth-Centered, Earth-Fixed Coordinates to
    # Geodetic Coordinates, IEEE Transactions on Aerospace and Electronic
    # Systems, 32 (1996) 473-476.
    w = math.sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1])
    z = ecef[2]
    zp = abs(z)
    w2 = w * w
    r2 = z * z + w2
    r  = math.sqrt(r2)
    s2 = z * z / r2
    c2 = w2 / r2
    u = a2 / r
    v = a3 - a4 / r
    if c2 > 0.3:
        s = (zp / r) * (1 + c2 * (a1 + u + s2 * v) / r)
        lat = math.asin(s)
        ss = s * s
        c = math.sqrt(1 - ss)
    else:
        c = (w / r) * (1 - s2 * (a5 - u - c2 * v) / r)
        lat = math.acos(c)
        ss = 1 - c * c
        s = math.sqrt(ss)
    g = 1 - e2 * ss
    rg = a / math.sqrt(g)
    rf = a6 * rg
    u = w - rg * c
    v = zp - rf * s
    f = c * u + s * v
    m = c * v - s * u
    p = m / (rf / g + f)
    lat = lat + p
    if z < 0:
        lat = -lat
    return (lat, math.atan2(ecef[1], ecef[0]), f + m * p / 2)
