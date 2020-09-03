import numpy as np
from adcs_lib import vector

'''This module is for our use of quaternions in implementating rotations.'''

def metric(p, q):
    '''
    Measures the distance between two quaternions. Symmetric function.

    Parameters
    ----------
    p : numpy.ndarray
        A quaternion.
    q : numpy.ndarray
        A quaternion.

    Returns
    -------
    float
        Distance between the two quaternions.
    '''
    return 1 - np.dot(p, q)

def xi(q):
    '''
    4x3 matrix representative of a quaternion acting on a vector by quaternionic multiplication.

    Parameters
    ----------
    q : numpy.ndarray
        A quaternion.

    Returns
    -------
    numpy.ndarray
        A matrix which acts on the left of a vector.
    '''
    return np.block([[-q[1:].T],
            [q[0] * np.eye(3) + vector.cross(q[1:])]
    ])

def conjugate(q):
    '''The conjugate of a unit quaternion is also its inverse. This is analogous to complex conjugates.

    Parameters
    ----------
    q : numpy.ndarray
        A quaternion.

    Returns
    -------
    numpy.ndarray
        Conjugate of the quaternion.
    '''
    return np.array([q[0], -q[1], -q[2], -q[3]])

def product(a, b):
    '''This is Hamilton's quaternion product. The real part of a quaternion is q[0].
    This is not a commutative operation, that is, product(a, b) != product(b, a).
    However, it is associative, i.e., product(product(a, b), c) == product(a, product(b, c)).
    Be aware that some authors use the opposite sign convention for the cross product term.


    Parameters
    ----------
    a : numpy.ndarray
        A quaternion.
    b : numpy.ndarray
        A quaternion.

    Returns
    -------
    numpy.ndarray
        The product of the two quaternions.
    '''
    v = b[0] * a[1:] + a[0] * b[1:] + np.cross(a[1:], b[1:])
    return np.array([a[0] * b[0] - np.dot(a[1:], b[1:]), v[0], v[1], v[2]])

def sandwich(q, v):
    '''This expresses a vector in the reference frame represented by the quaternion.
    It is a coordinate transformation, not a rotation.
    In other words, this takes inertial vectors and tells you their coordinates in the body frame.

    Parameters
    ----------
    q : numpy.ndarray
        A quaternion.
    v : numpy.ndarray
        A vector in the coordinate frame the quaternion is with respect to.

    Returns
    -------
    numpy.ndarray
        A vector in the coordinate frame of the quaternion.
    '''
    return product(conjugate(q), product(np.array([0, v[0], v[1], v[2]]), q))[1:]

def sandwich_opp(q, v):
    '''This expresses a vector in the reference frame that the quaternion is with respect to.
    It is a coordinate transformation, not a rotation.
    In other words, this takes body vectors and tells you their coordinates in the inertial frame.

    Parameters
    ----------
    q : numpy.ndarray
        A quaternion.
    v : numpy.ndarray
        A vector in the coordinate frame represented by the quaternion.

    Returns
    -------
    numpy.ndarray
        A vector in the coordinate frame that the quaternion is with respect to.
    '''
    return sandwich(conjugate(q), v)

def error_quat(q, q_ref):
    '''Calculates attitude error. Refer to (Wie et al, 1988)

    Parameters
    ----------
    q : numpy.ndarray
        Current attitude.
    q_ref : numpy.ndarray
        Desired attitude.

    Returns
    -------
    numpy.ndarray
        Error quaternion between current and desired orientations
    '''
    return product(conjugate(q_ref), q)

def axisangle_to_quat(r, theta):
    '''Encodes an axis-angle representation of a coordinate transformation or rotation as a unit quaternion.

    Parameters
    ----------
    r : numpy.ndarray
        Unit vector axis of rotation.
    theta : float
        Angle of rotation.

    Returns
    -------
    numpy.ndarray
        Quaternion representation of rotation.
    '''
    v = np.sin(theta/2) * r
    return np.array([np.cos(theta/2), v[0], v[1], v[2]])

def eulerangle_to_quat(RA, dec, roll):
    '''Encodes star tracker's attitude representation as a quaternion in
    3-2-1 order (yaw, pitch, roll). This quaternion transforms the inertial frame to the body frame.

    Parameters
    ----------
    RA : float
        Right ascenscion.
    dec : float
        Declination.
    roll : float
        Roll.

    Returns
    -------
    numpy.ndarray
        Quaternion representation of attitude.
    '''
    RA = np.radians(RA) / 2
    dec = np.radians(dec) / 2
    roll = np.radians(roll) / 2
    c_phi = np.cos(RA)
    s_phi = np.sin(RA)
    c_theta = np.cos(dec)
    s_theta = np.sin(dec)
    c_psi = np.cos(roll)
    s_psi = np.sin(roll)
    return np.array([c_phi * c_theta * c_psi  +  s_phi * s_theta * s_psi,
                     c_phi * c_theta * s_psi  -  s_phi * s_theta * c_psi,
                     c_phi * s_theta * c_psi  +  s_phi * c_theta * s_psi,
                     s_phi * c_theta * c_psi  -  c_phi * s_theta * s_psi])

def quat_to_startracker(q):
    '''Convert quaternion attitude to RA, DEC, ROLL. I haven't rigorously proved this works yet.
    Reference http://general-tools.cosmos.esa.int/iso/manuals/HANDBOOK/gen_hb/node87.php and check later!

    Parameters
    ----------
    q : numpy.ndarray
        A quaternion.

    Returns
    -------
    list
        Floats for right ascenscion, declination, and roll.
    '''
    dec = np.arcsin(2 * (q[1]*q[3] - q[2]*q[0]))
    cos_dec = np.cos(dec)
    ra = np.arcsin(2 * (q[1]*q[2] + q[3]*q[0]) / cos_dec)
    roll = np.arcsin(2 * (q[1]*q[0] + q[3]*q[2]) / cos_dec)
    return [ra, dec, roll]

def path_lift(RA, dec, roll, q_ref):
    '''The most computationally efficient way to do this is to initially store q_ref=(1,0,0,0) in filter.
    Then take 1-q0 of the startracker measurement and if it's below a threshold, use measurement.
    Otherwise, store q_ref=(-1,0,0,0), take opposite of measurement, and compare to 1+q0 until next switch.
    However, this should work just fine.
    I'm still not sure whether we'd need to remove the sgn(q0) term from the control if we lift the path.
    For details please refer to Mayhew, et al 2013.

    Parameters
    ----------
    RA : float
        Right ascenscion.
    dec : float
        Declination.
    roll : float
        Roll.
    q_ref : numpy.ndarray
        Last known attitude.

    Returns
    -------
    numpy.ndarray
        Quaternion representation of attitude, closest to last attitude.
    '''
    q = eulerangle_to_quat(RA, dec, roll)
    if metric(q, q_ref) < metric(-q, q_ref):
        return q
    else:
        return -q
