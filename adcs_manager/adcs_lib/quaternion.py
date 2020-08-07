import numpy as np

'''This module is for our use of quaternions in implementating rotations.'''

def conjugate(q):
    '''The conjugate of a unit quaternion is also its inverse.
    :returns: Conjugate quaternion'''
    return np.array([q[0], -q[1], -q[2], -q[3]])

def normalize(q):
    '''This enforces the unit norm constraint for quaternions or vectors'''
    return q / np.linalg.norm(q)

def product(a, b):
    '''This is Hamilton's quaternion product. The real part of a quaternion is q[0].
    This is not a symmetric operation, that is, product(a, b) != product(b, a).
    Be aware that some aerospace textbooks use the opposite sign convention for the cross product term.

    :params: Two quaternions.
    :returns: The product of the two quaternions.'''
    v = b[0] * a[1:] + a[0] * b[1:] + np.cross(a[1:], b[1:])
    return np.array([a[0] * b[0] - np.dot(a[1:], b[1:]), v[0], v[1], v[2]])

def sandwich(q, v):
    '''This expresses a vector in the reference frame represented by the quaternion.
    It is a coordinate transformation, not a rotation.

    :params: quaternion, vector.
    :returns: vector.
    '''
    return product(conjugate(q), product(np.array([0, v[0], v[1], v[2]]), q))[1:]

def error_quat(q, q_ref):
    '''Calculates attitude error. Refer to (Wie et al, 1988)

    :params: Current attitude, desired attitude.
    :returns: Error quaternion between current and desired orientations'''
    mat = np.array([[q_ref[0], q_ref[1], q_ref[2], q_ref[3]],
                    [-q_ref[1], q_ref[0], q_ref[3], -q_ref[2]],
                    [-q_ref[2], -q_ref[3], q_ref[0], q_ref[1]],
                    [-q_ref[3], q_ref[2], -q_ref[1], q_ref[0]]])
    return mat.dot(q)

def axisangle_to_quat(r, theta):
    '''Encodes an axis-angle representation of a rotation as a unit quaternion.

    :params: Axis of rotation, angle of rotation.
    :returns: Quaternion representation of rotation.'''
    v = np.sin(theta/2) * r
    return np.array([np.cos(theta/2), v[0], v[1], v[2]])

def eulerangle_to_quat(RA, dec, orientation):
    '''Encodes star tracker's attitude representation as a quaternion in
    3-2-1 order (yaw, pitch, roll). This quaternion transforms the inertial frame to the body frame.

    :params: right ascenscion, declination, roll.
    :returns: Quaternion representation of attitude.'''
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
    dec = np.arcsin(2 * (q[1]*q[3] - q[2]*q[0]))
    cos_dec = np.cos(dec)
    ra = np.arcsin(2 * (q[1]*q[2] + q[3]*q[0]) / cos_dec)
    roll = np.arcsin(2 * (q[1]*q[0] + q[3]*q[2]) / cos_dec)
    return [ra, dec, roll]
