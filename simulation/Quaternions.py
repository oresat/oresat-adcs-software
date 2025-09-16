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

# def quat_mult(q_rot, q_init): # Hamiltonian quaternion multiplcation
#     x1, y1, z1, s1 = q_rot
#     x2, y2, z2, s2 = q_init
#     q_new = [
#         s1*x2 + x1*s2 + y1*z2 - z1*y2,
#         s1*y2 - x1*z2 + y1*s2 + z1*x2,
#         s1*z2 + x1*y2 - y1*x2 + z1*s2,
#         s1*s2 - x1*x2 - y1*y2 - z1*z2

#     ]
#     return normalize(q_new)

def quat_mult(q_rot, q_init): # Shuster = Hamilton with the cross-product term NEGATED. This means that the inputs are also flipped. They are not flipped yet for testing purposes
    x1, y1, z1, s1 = q_rot
    x2, y2, z2, s2 = q_init
    
    q_new = [
        s1*x2 + x1*s2 - y1*z2 + z1*y2,
        s1*y2 + y1*s2 - z1*x2 + x1*z2,
        s1*z2 + z1*s2 - x1*y2 + y1*x2,
        s1*s2 - x1*x2 - y1*y2 - z1*z2
    ]
    return normalize(q_new)

def quat_error(q_target, q_current): #4 error defined by Markley & Crassidis. Semi stable with hamilton for about 100 seconds
    return quat_mult(q_current, quat_conjugate(q_target)) # returns normalized quaternion. Sanitization happens in quat_mult function

# def quat_error(q_target, q_current): # This one works without negating the A matrix when using hamilton 
#     return quat_mult(quat_conjugate(q_target), q_current) # return normalized quaternion. Sanitization happens in quat_mult function

# def quat_error(q_target, q_current): #2 is the inverse of #4, flipping A matrix to negative sign with hamilton makes this behave the same as #4
#     return quat_mult(q_target, quat_conjugate(q_current)) # return normalized quaternion. Sanitization happens in quat_mult function

# def quat_error(q_target, q_current): #3 THIS ONE SEEMS TO WORK FOR SOME REASON WITH SMALL ANGLE ROTATIONS AND NEGATIVE A MATRIX
#     return quat_mult(quat_conjugate(q_current), q_target) # returns normalized quaternion. Sanitization happens in quat_mult function


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

def quat_to_axis(q_error): # returns axis of rotation
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

def rotate_vec_by_quat(v, q):
    """Rotate 3-vector v by unit, scalar-last quaternion q = [ex,ey,ez, s]."""
    ex, ey, ez, s = q
    e = np.array([ex, ey, ez], dtype=float)
    v = np.array(v, dtype=float)
    t = 2.0 * np.cross(e, v)
    return v + s * t + np.cross(e, t)

if __name__ == "__main__":
    qtarget = axis_angle_to_quaternion([1,1,0], 90)
    print(qtarget)
    qcurrent = [0,0,0,1]
    q_error = quat_error(qtarget, qcurrent)
    print(error_angle(q_error))
    
    qtarget = axis_angle_to_quaternion([1,1,0], 90)
    print(qtarget)
    qcurrent = axis_angle_to_quaternion([1,0,0], 45)
    # q_error = quat_error(qtarget, qcurrent)
    # print(q_error, error_angle(q_error))
    q_error = quat_error(qtarget, qcurrent)
    print(q_error, error_angle(q_error))
    
    print(quat_conjugate(axis_angle_to_quaternion([0,1,0], 180)))
    
    
