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

def quat_from_dcm_scalar_last(self, C_BN):
    """
    C_BN: DCM mapping inertial → body using numerically stable methods which
    avoid singularities and square roots of negative values
    Returns scalar-last quaternion
    
    qw represents scalar component of quaternion, as s is already used as
    intermediate variable
    """
    m = C_BN
    trace = np.trace(m)

    if trace > 0.0: # if trace is greater than zero, start with scalar component
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (m[2,1] - m[1,2]) * s
        qy = (m[0,2] - m[2,0]) * s
        qz = (m[1,0] - m[0,1]) * s
    else:
        if m[0,0] > m[1,1] and m[0,0] > m[2,2]:
            s = 2.0 * np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2])
            qw = (m[2,1] - m[1,2]) / s
            qx = 0.25 * s
            qy = (m[0,1] + m[1,0]) / s
            qz = (m[0,2] + m[2,0]) / s
        elif m[1,1] > m[2,2]:
            s = 2.0 * np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2])
            qw = (m[0,2] - m[2,0]) / s
            qx = (m[0,1] + m[1,0]) / s
            qy = 0.25 * s
            qz = (m[1,2] + m[2,1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1])
            qw = (m[1,0] - m[0,1]) / s
            qx = (m[0,2] + m[2,0]) / s
            qy = (m[1,2] + m[2,1]) / s
            qz = 0.25 * s

    q = np.array([qx, qy, qz, qw])
    q = q / np.linalg.norm(q)
    return q

def quat_from_cartesian_vector(target, eps=1e-8):
    target = target / np.linalg.norm(target) # normalize target vector just in case rotation introduced numerical noise
    z = [0.0, 0.0, 1.0] # target facing of satellite will be z vector (to be rotated by control software later if Cirrus Flux Camera is used instead of Selfie Camera)
    d = np.dot(target, z)

    if d > 1.0 - eps:
        return np.array([0.0, 0.0, 0.0, 1.0])

    if d < -1.0 + eps:
        ortho = np.array([1.0, 0.0, 0.0])
        if abs(target[0]) > 0.9:
            ortho = np.array([0.0, 1.0, 0.0])
        axis = np.cross(target, ortho)
        axis = axis / np.linalg.norm(axis)
        return np.array([axis[0], axis[1], axis[2], 0.0])

    c = np.cross(z, target)
    s = np.sqrt((1.0 + d) * 2.0)
    q_xyz = c / s
    q_w   = 0.5 * s
    q = np.array([q_xyz[0], q_xyz[1], q_xyz[2], q_w])
    return q / np.linalg.norm(q)