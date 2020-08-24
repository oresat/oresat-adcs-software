import numpy as np

'''This module is for our use of quaternions in implementating rotations.'''

def saturation(x, x_max):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    largest_x = np.max(np.abs(x))
    if largest_x > x_max:
        x = x_max * x / largest_x
    return x


def metric(p, q):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    return 1 - np.dot(p, q)

def cross(v):
    '''
    Returns matrix equivalent to taking cross product of vector from left.'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    return np.array([[0, -v[2], v[1]],
                      [v[2], 0, -v[0]],
                      [-v[1], v[0], 0]])

def xi(q):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    return np.block([[-q[1:].T],
            [q[0] * np.eye(3) + cross(q[1:])]
    ])

def conjugate(q):
    '''The conjugate of a unit quaternion is also its inverse.
    :returns: Conjugate quaternion'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    return np.array([q[0], -q[1], -q[2], -q[3]])

def normalize(q):
    '''This enforces the unit norm constraint for quaternions or vectors'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    return q / np.linalg.norm(q)

def product(a, b):
    '''This is Hamilton's quaternion product. The real part of a quaternion is q[0].
    This is not a symmetric operation, that is, product(a, b) != product(b, a).
    Be aware that some aerospace textbooks use the opposite sign convention for the cross product term.

    :params: Two quaternions.
    :returns: The product of the two quaternions.'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    v = b[0] * a[1:] + a[0] * b[1:] + np.cross(a[1:], b[1:])
    return np.array([a[0] * b[0] - np.dot(a[1:], b[1:]), v[0], v[1], v[2]])

def sandwich(q, v):
    '''This expresses a vector in the reference frame represented by the quaternion.
    It is a coordinate transformation, not a rotation.

    :params: quaternion, vector.
    :returns: vector.
    '''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    return product(conjugate(q), product(np.array([0, v[0], v[1], v[2]]), q))[1:]

def sandwich_opp(q, v):
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    return sandwich(conjugate(q), v)

def error_quat(q, q_ref):
    '''Calculates attitude error. Refer to (Wie et al, 1988)

    :params: Current attitude, desired attitude.
    :returns: Error quaternion between current and desired orientations'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    return product(conjugate(q_ref), q)
    mat = np.array([[q_ref[0], q_ref[1], q_ref[2], q_ref[3]],
                    [-q_ref[1], q_ref[0], q_ref[3], -q_ref[2]],
                    [-q_ref[2], -q_ref[3], q_ref[0], q_ref[1]],
                    [-q_ref[3], q_ref[2], -q_ref[1], q_ref[0]]])
    return mat.dot(q)

def axisangle_to_quat(r, theta):
    '''Encodes an axis-angle representation of a rotation as a unit quaternion.

    :params: Axis of rotation, angle of rotation.
    :returns: Quaternion representation of rotation.'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    v = np.sin(theta/2) * r
    return np.array([np.cos(theta/2), v[0], v[1], v[2]])

def eulerangle_to_quat(RA, dec, orientation):
    '''Encodes star tracker's attitude representation as a quaternion in
    3-2-1 order (yaw, pitch, roll). This quaternion transforms the inertial frame to the body frame.

    :params: right ascenscion, declination, roll.
    :returns: Quaternion representation of attitude.'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    RA = np.radians(RA) / 2
    dec = np.radians(dec) / 2
    ortn = np.radians(orientation) / 2
    c_phi = np.cos(RA)
    s_phi = np.sin(RA)
    c_theta = np.cos(dec)
    s_theta = np.sin(dec)
    c_psi = np.cos(ortn)
    s_psi = np.sin(ortn)
    return np.array([c_phi * c_theta * c_psi  +  s_phi * s_theta * s_psi,
                     c_phi * c_theta * s_psi  -  s_phi * s_theta * c_psi,
                     c_phi * s_theta * c_psi  +  s_phi * c_theta * s_psi,
                     s_phi * c_theta * c_psi  -  c_phi * s_theta * s_psi])

def quat_to_startracker(q):
    '''Convert quaternion attitude to RA, DEC, ROLL. I haven't rigorously proved this works yet.
    Reference http://general-tools.cosmos.esa.int/iso/manuals/HANDBOOK/gen_hb/node87.php and check later.

    :params: quaternion.
    :returns: right ascenscion, declination, roll'''
    '''

    Parameters
    ----------

    Returns
    -------
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
    I'm still not sure whether we'd need to remove the sgn(q0) term from the control if we lift the path.'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    q = eulerangle_to_quat(RA, dec, roll)
    if metric(q, q_ref) < metric(-q, q_ref):
        return q
    else:
        return -q
