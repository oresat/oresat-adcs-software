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

def quat_from_dcm_scalar_last(m):
    """
    m (C_BN matrix): DCM mapping inertial -> body using numerically stable methods which
    avoid singularities and square roots of negative values
    Returns scalar-last quaternion
    
    qs represents scalar component of quaternion
    
    Conversion definition based on work in:
    "Quaternion to DCM and Back Again" by Kurt A. Motekew
    """
    trace = np.trace(m)
    
    if ((trace > m[0, 0]) and (trace > m[1, 1]) and (trace > m[2, 2])):
        qs = np.sqrt((1.0 + m[0,0] + m[1,1] + m[2,2])/4)
        qx = (m[1, 2] - m[2, 1])/(4*qs)
        qy = (m[2, 0] - m[0, 2])/(4*qs)
        qz = (m[0, 1] - m[1, 0])/(4*qs)
        
    elif ((m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2])):
        qx = np.sqrt((1.0 + m[0,0] - m[1,1] - m[2,2])/4)
        qs = (m[1, 2] - m[2, 1])/(4*qx)
        qz = (m[2, 0] + m[0, 2])/(4*qx)
        qy = (m[0, 1] + m[1, 0])/(4*qx)
        
    elif ((m[1, 1] > m[2, 2])):
        qy = np.sqrt((1.0 - m[0,0] + m[1,1] - m[2,2])/4)
        qz = (m[1, 2] + m[2, 1])/(4*qy)
        qs = (m[2, 0] - m[0, 2])/(4*qy)
        qx = (m[0, 1] + m[1, 0])/(4*qy)
        
    else:
        qz = np.sqrt((1.0 - m[0,0] - m[1,1] + m[2,2])/4)
        qy = (m[1, 2] + m[2, 1])/(4*qz)
        qx = (m[2, 0] + m[0, 2])/(4*qz)
        qs = (m[0, 1] - m[1, 0])/(4*qz)
    
    q = np.array([qx, qy, qz, qs])
    q = q / np.linalg.norm(q)
    return q