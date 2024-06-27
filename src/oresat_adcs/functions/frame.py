import numpy as np
from . import quaternion, vector
import math

'''These functions are for transformations between coordinate systems'''

def inertial_to_ecef(clock):
    '''
    Transformation from inertial frame to ECEF. Cartesian x, y, z for both.

    Parameters
    ----------
    clock : jday.Clock
        These reference frames differ by rotation around z-axis depending on what time it is.

    Returns
    -------
    numpy.ndarray
        Matrix for coordinate transformation from inertial frame to ECEF frame.
    '''
    gmst = clock.theta_gmst()
    C = np.cos(gmst)
    S = np.sin(gmst)
    return np.array([[C, S, 0],
                     [-S, C, 0],
                     [0, 0, 1]])

def lvlh_to_inertial(r_I, v_I):
    '''
    Transformation from local-vertical local-horizontal orbital frame to inertial frame.

    Parameters
    ----------
    r_I : numpy.ndarray
        Position of satellite in inertial frame.
    v_I : numpy.ndarray
        Velocity of satellite in inertial frame.

    Returns
    -------
    numpy.ndarray
        Matrix for coordinate transformation from local-vertical local-horizontal frame to inertial frame.
    '''
    o_3I = - vector.normalize(r_I)
    o_2I = - vector.normalize(np.cross(r_I, v_I))
    o_1I = np.cross(o_2I, o_3I)
    return np.array([[o_1I[0], o_2I[0], o_3I[0]],
                     [o_1I[1], o_2I[1], o_3I[1]],
                     [o_1I[2], o_2I[2], o_3I[2]]])

def geodetic_to_ECEF(lat, long, h):
    '''
    Coordinate transformation from WGS-84 geodetic coordinates to ECEF geocentric coordinates.
    Might not need this if GPS is already in ECEF.

    Parameters
    ----------
    lat : float
        Latitude.
    long : float
        Longitude.
    h : float
        Vertical distance above geode along surface normal.

    Returns
    -------
    numpy.ndarray
        Position of satellite in ECEF frame.
    '''
    a = 6378137.0 # m, semimajor axis
    e = 0.0818 # eccentricity approximation
    N = a / np.sqrt(1 - (e*np.sin(lat))**2)
    temp = (N + h) * np.cos(lat)
    x = temp * np.cos(long)
    y = temp * np.sin(long)
    z = (N*(1 - e**2) + h) * np.sin(lat)
    return np.array([x, y, z])

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

def ecef_to_lla(ecef):
    '''Convert ECEF (meters) coordinates to geodetic coordinates (degrees and meters).
    It would be good to check out the OG source and make sure this is appropriate.
    Uses those parameters right above here.

    Sourced from https://possiblywrong.wordpress.com/2014/02/14/when-approximate-is-better-than-exact/
    Originally from
    Olson, D. K., Converting Earth-Centered, Earth-Fixed Coordinates to
    Geodetic Coordinates, IEEE Transactions on Aerospace and Electronic
    Systems, 32 (1996) 473-476.

    Parameters
    ----------
    ecef : numpy.ndarray
        Position of satellite in ECEF frame.

    Returns
    -------
    list
        Latitude, longitude, and vertical distance above geode along surface normal.
    '''
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
    return (np.degrees(lat), np.degrees(math.atan2(ecef[1], ecef[0])), f + m * p / 2)
