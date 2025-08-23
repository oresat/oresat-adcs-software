'''
Quaternion tools and operations

q = [x, y, z, s], with s = scalar (NASA/JPL notation)
'''

import numpy as np

def quat_conjugate(q):
    q1, q2, q3, qs = q
    return [-q1, -q2, -q3, qs]

def hemi(q):
    q = np.asarray(q, dtype=float)
    return q if q[3] >= 0 else -q # if scalar part negative negate entire quaternion

def normalize(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)

def quat_mult(q_rot, q_init):
    x1, y1, z1, s1 = normalize(hemi(q_rot))    # sanitize inputs by normalizing and checking signs (hemisphere)
    x2, y2, z2, s2 = normalize(hemi(q_init))   # sanitize inputs by normalizing and checking signs (hemisphere)
    q_new = [
        s1*x2 + x1*s2 + y1*z2 - z1*y2,
        s1*y2 - x1*z2 + y1*s2 + z1*x2,
        s1*z2 + x1*y2 - y1*x2 + z1*s2,
        s1*s2 - x1*x2 - y1*y2 - z1*z2 # CHECK THIS. MOVED THIS LINE FROM FIRST TO LAST POSITION FOR REORDERING TO NASA/JPL NOTATION

    ]
    q_new = hemi(q_new) # if scalar part negative negate entire quaternion
    return normalize(q_new)

def quat_error(q_target, q_current):
    return quat_mult(q_target, quat_conjugate(q_current)) # return normalized quaternion. Sanitization happens in quat_mult function

def to_scalar_last(q): # convert quaternion to scalar-last convention
    return np.concatenate((q[1:], [q[0]]))

def axis_angle_to_quaternion(axis, angle_deg):
    angle_rad = np.radians(angle_deg)
    u = np.array(axis) / np.linalg.norm(axis)
    q_vec = u * np.sin(angle_rad / 2)
    q_scalar = np.cos(angle_rad / 2)
    q = np.concatenate((q_vec, [q_scalar])) # create scalar-last quaternion 
    q = q/np.linalg.norm(q) # normalize quaternion to account for numerical errors
    return q

def quat_to_angle(q_error):
    q = q_error / np.linalg.norm(q_error)
    w = q[3]
    sin_half_angle = np.sqrt(1 - w**2)
    if sin_half_angle < 1e-6:
        axis = np.array([1.0, 0.0, 0.0])  # default axis
    else:
        axis = q[:3] / sin_half_angle
    return axis

def error_angle(q_error):
    """
    Returns the error angle in degrees based on error quaternion

    Parameters:
    q_error : error quaternion

    Returns:
    Error angle in degrees
    """
    
    return 2*np.acos(abs(q_error[3])) * 180/ np.pi

def rotation_sequence_to_quaternion(angles_deg, order='xyz'):
    """
    Convert a sequence of body-axis rotations into a single quaternion. Should be purely for artificial testing environment.

    Parameters:
    angles_deg: list or tuple of 3 angles [x, y, z] in degrees
    order: str, order of rotations, e.g. 'xyz', 'zyx', etc.

    Returns:
    4x1 unit quaternion in scalar-last format [x, y, z, s]
    """
    assert len(angles_deg) == 3, "improper number of arguments in angles_deg vector"
    assert len(order) == 3 and all(c in 'xyz' for c in order), "check order string"

    # Axis map
    axis_map = {'x': [1, 0, 0], 'y': [0, 1, 0], 'z': [0, 0, 1]}
    
    # Generate individual quaternions
    q_list = [axis_angle_to_quaternion(axis_map[ax], angle)
              for ax, angle in zip(order, angles_deg)]

    # Compose in reverse (for extrinsic/global rotations)
    q_total = q_list[2]
    q_total = quat_mult(q_total, q_list[1])
    q_total = quat_mult(q_total, q_list[0])
    
    # Normalize
    q_total /= np.linalg.norm(q_total)
    assert np.linalg.norm(q_total) == 1, "Created quaternion norm not equal to 1" # check that quaternion was properly assembled
    return q_total

def get_rotation_angle_from_quaternion(q, degrees=False):
    """
    Returns the magnitude of rotation represented by a unit quaternion.

    Parameters:
    q : array-like of shape (4,)
        Unit quaternion in scalar-last format [x, y, z, s]
    degrees : bool
        If True, returns angle in degrees

    Returns:
    float : rotation angle
    """
    q = np.array(q)
    q = q / np.linalg.norm(q)  # ensure it's normalized
    q0 = q[3]  # scalar part
    angle = 2 * np.arccos(np.clip(q0, -1.0, 1.0))  # clip for numerical safety

    return np.degrees(angle) if degrees else angle

if __name__ == "__main__":
    qtarget = axis_angle_to_quaternion([0,1,0], 90)
    print(qtarget)
    qtarget = quat_mult(axis_angle_to_quaternion([1,0,0], 90), qtarget)
    print(qtarget)
    qcurrent = [0,0,0,1]
    q_error = quat_error(qtarget, qcurrent)
    print(error_angle(q_error))