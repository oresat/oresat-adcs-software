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

def quat_mult(q_rot, q_init): # Shuster quaternion multiplication
    x1, y1, z1, s1 = q_rot
    x2, y2, z2, s2 = q_init
    
    q_new = [
        s1*x2 + x1*s2 - y1*z2 + z1*y2,
        s1*y2 + y1*s2 - z1*x2 + x1*z2,
        s1*z2 + z1*s2 - x1*y2 + y1*x2,
        s1*s2 - x1*x2 - y1*y2 - z1*z2
    ]
    return normalize(q_new)

def quat_error(q_target, q_current): # error defined by Markley & Crassidis.
    return quat_mult(q_current, quat_conjugate(q_target)) # returns normalized quaternion. Sanitization happens in quat_mult function

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

if __name__ == "__main__":
    r1 = axis_angle_to_quaternion([1,0,0], -90)
    r2 = axis_angle_to_quaternion([0,1,0], 60)
    
    r_tot = quat_mult(r2, r1)
    axis = quat_to_axis(r_tot)
    angle = error_angle(quat_error(r_tot, [0,0,0,1]))
    
    
    
