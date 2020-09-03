import numpy as np

def saturation(x, x_max):
    '''
    Saturation function for constrained controls.

    Parameters
    ----------
    x : numpy.ndarray
        Input vector in question.
    x_max : float
        Maximum allowable value for inputs.

    Returns
    -------
    numpy.ndarray
        Vector limited by saturation.
    '''
    largest_x = np.max(np.abs(x))
    if largest_x > x_max:
        x = x_max * x / largest_x
    return x

def cross(v):
    '''
    Returns the matrix equivalent to taking the cross product of a vector from the left.

    Parameters
    ----------
    v : numpy.ndarray
        The vector acting on the left by cross product.

    Returns
    -------
    numpy.ndarray
        Skew-symmetric matrix which performs cross product when acting on left.
    '''
    return np.array([[0, -v[2], v[1]],
                      [v[2], 0, -v[0]],
                      [-v[1], v[0], 0]])

def normalize(q):
    '''This enforces the unit norm constraint for quaternions or vectors.

    Parameters
    ----------
    q : numpy.ndarray
        Array to normalize.

    Returns
    -------
    numpy.ndarray
        Normalized array.
    '''
    norm = np.linalg.norm(q)
    if norm == 0:
        q_hat = q
    else:
        q_hat = q / norm
    return q_hat

def pointing_vector(position, target):
    '''
    Unit pointing vector.

    Parameters
    ----------
    position : numpy.ndarray
        3D array of floats for inertial frame position data.
    target : numpy.ndarray
        3D array of floats for inertial frame target location data.

    Returns
    -------
    numpy.ndarray
        A unit vector which points from the satellite's position to a target location.
    '''

    p = target - position
    return normalize(p)

def R_axis_angle(boresight, point):
    '''
    Rotate one vector into another.

    Parameters
    ----------
    boresight : numpy.ndarray
        3D array of floats for inertial frame camera boresight data.
    point : numpy.ndarray
        3D array of floats for inertial frame pointing vector data.

    Returns
    -------
    numpy.ndarray
        3D array for inertial frame axis of rotation.
    float
        Angle of rotation, with usual right-handed convention.
    '''
    r     = normalize(np.cross(boresight, point))
    # clip is to keep floating pt error from crashing arccos
    theta = np.arccos(np.clip(np.dot(boresight, point), -1, 1))
    return r, theta # use + for active (- for passive)
