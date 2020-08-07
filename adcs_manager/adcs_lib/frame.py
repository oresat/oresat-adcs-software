import numpy as np
<<<<<<< HEAD:adcs_lib/frame.py
import quaternion
||||||| 0b886c4:adcs_lib/frame.py
=======
from adcs_lib import quaternion
>>>>>>> adcs_daemon:adcs_manager/adcs_lib/frame.py

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
    o_3I = - quaternion.normalize(r_I)
    o_2I = - quaternion.normalize(np.cross(r_I, v_I))
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
