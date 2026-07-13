"""
Quaternion tools and operations

q = [x, y, z, s], with s = scalar (NASA/JPL notation)
"""

from typing import Sequence, Union

import numpy as np

ArrayLike = Union[Sequence[float], np.ndarray]


def conjugate(quat, scalar_last=False) -> np.ndarray:
    """Compute the conjugate of a quaternion.

    Paramaters
    ----------
    quat
        quaternion
    scalar_last
        set to true if input and output should be scalar last
    """
    qs, qi, qj, qk = quat if not scalar_last else np.roll(quat, -1)
    return (
        np.array([qs, -qi, -qj, -qk])
        if not scalar_last
        else np.array([-qi, -qj, -qk, qs])
    )


def quat_conjugate(q: ArrayLike) -> np.ndarray:
    """Compute the conjugate of a quaternion

    Parameters
    ----------
    q : ArrayLike
        The quaternion to conjugate

    Returns
    -------
    np.ndarray
        The conjugate of q
    """
    q1, q2, q3, qs = q
    return np.array([-q1, -q2, -q3, qs])


def hemi(q: ArrayLike) -> np.ndarray:
    """Quaternion hemisphere check: ensure the scalar component is non-negative

    Parameters
    ----------
    q : ArrayLike
        The quaternion to check

    Returns
    -------
    np.ndarray
        The quaternion, if already correct, else its negation
    """
    q: np.ndarray = np.asarray(q, dtype=float)
    return q if q[3] >= 0 else -q  # if scalar part negative negate entire quaternion


def normalize(q: ArrayLike) -> np.ndarray:
    """Normalize a quaternion

    Parameters
    ----------
    q : ArrayLike
        The quaternion to normalize

    Returns
    -------
    np.ndarray
        ``q``, normalized
    """
    q: np.ndarray = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def quat_mult(q_rot: ArrayLike, q_init: ArrayLike) -> np.ndarray:
    r"""Shuster quaternion multiplication

    Parameters
    ----------
    q_rot : ArrayLike
        The quaternion representing the rotation to apply
    q_init : ArrayLike
        The quaternion to be rotated

    Returns
    -------
    np.ndarray
        :math:`q_rot \otimes q_init` (left multiplication by q_rot),
        representing the rotation of q_init by q_rot
    """
    x1, y1, z1, s1 = q_rot
    x2, y2, z2, s2 = q_init

    q_new = [
        s1 * x2 + x1 * s2 - y1 * z2 + z1 * y2,
        s1 * y2 + y1 * s2 - z1 * x2 + x1 * z2,
        s1 * z2 + z1 * s2 - x1 * y2 + y1 * x2,
        s1 * s2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]
    return normalize(q_new)


def quat_error(q_target: ArrayLike, q_current: ArrayLike) -> np.ndarray:
    """Calculate the error between two quaternions, as defined by Markley & Crassidis.
    This function automatically sanitizes by performing normalization and hemisphere checks.

    Parameters
    ----------
    q_target
        The target orientation quaternion
    q_current
        The current orientation quaternion
    Returns
    -------
    np.ndarray
        The error quaternion
    """
    # returns normalized quaternion. Sanitization happens in quat_mult function
    return quat_mult(q_current, quat_conjugate(q_target))


def to_scalar_last(q: ArrayLike) -> np.ndarray:
    """Convert quaternion from scalar-first to scalar-last convention

    Parameters
    ----------
    q : np.ndarray
        The scalar-first quaternion
    Returns
    -------
    np.ndarray
        q, converted to scalar-last form
    """
    return np.concatenate((q[1:], [q[0]]))


def axis_angle_to_quaternion(axis: ArrayLike, angle_deg: float) -> np.ndarray:
    angle_rad = np.radians(angle_deg)
    u = np.array(axis) / np.linalg.norm(axis)
    q_vec = u * np.sin(angle_rad / 2)
    q_scalar = np.cos(angle_rad / 2)
    q = np.concatenate((q_vec, [q_scalar]))  # create scalar-last quaternion
    q = q / np.linalg.norm(q)  # normalize quaternion to account for numerical errors
    return q


def quat_to_axis(q_error: np.ndarray) -> np.ndarray:
    """Calculate the axis of rotation from a quaternion

    Parameters
    ----------
    q_error : np.ndarray
        The quaternion

    Returns
    -------
    np.ndarray
        The axis of rotation
    """
    q = q_error / np.linalg.norm(q_error)
    w = q[3]
    sin_half_angle = np.sqrt(1 - w**2)
    if sin_half_angle < 1e-6:
        axis = np.array([1.0, 0.0, 0.0])  # default axis
    else:
        axis = q[:3] / sin_half_angle
    return axis


def error_angle(q_error) -> np.ndarray:
    """
    Returns the error angle in degrees based on error quaternion

    Parameters:
    q_error : error quaternion

    Returns:
    Error angle in degrees
    """

    return 2 * np.arccos(abs(q_error[3])) * 180 / np.pi


def quat_from_dcm_scalar_last(m: np.ndarray) -> np.ndarray:
    """DCM mapping inertial -> body using numerically stable methods which
    avoid singularities and square roots of negative values

    qs represents scalar component of quaternion

    Conversion definition based on work in:
    "Quaternion to DCM and Back Again" by Kurt A. Motekew

    Parameters
    ----------
    m : np.ndarray
        C_BN matrix to converting

    Returns
    -------
        Scalar-last quaternion
    """
    trace = np.trace(m)

    if (trace > m[0, 0]) and (trace > m[1, 1]) and (trace > m[2, 2]):
        qs = np.sqrt((1.0 + m[0, 0] + m[1, 1] + m[2, 2]) / 4)
        qx = (m[1, 2] - m[2, 1]) / (4 * qs)
        qy = (m[2, 0] - m[0, 2]) / (4 * qs)
        qz = (m[0, 1] - m[1, 0]) / (4 * qs)
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        qx = np.sqrt((1.0 + m[0, 0] - m[1, 1] - m[2, 2]) / 4)
        qs = (m[1, 2] - m[2, 1]) / (4 * qx)
        qz = (m[2, 0] + m[0, 2]) / (4 * qx)
        qy = (m[0, 1] + m[1, 0]) / (4 * qx)
    elif m[1, 1] > m[2, 2]:
        qy = np.sqrt((1.0 - m[0, 0] + m[1, 1] - m[2, 2]) / 4)
        qz = (m[1, 2] + m[2, 1]) / (4 * qy)
        qs = (m[2, 0] - m[0, 2]) / (4 * qy)
        qx = (m[0, 1] + m[1, 0]) / (4 * qy)
    else:
        qz = np.sqrt((1.0 - m[0, 0] - m[1, 1] + m[2, 2]) / 4)
        qy = (m[1, 2] + m[2, 1]) / (4 * qz)
        qx = (m[2, 0] + m[0, 2]) / (4 * qz)
        qs = (m[0, 1] - m[1, 0]) / (4 * qz)

    q = np.array([qx, qy, qz, qs])
    q = q / np.linalg.norm(q)
    return q


def hamiltonian(a_quat: np.ndarray, b_quat: np.ndarray, scalar_last: bool = False):
    """Compute the hamiltonian product of two quaternions.

    Note the cross product of the vector is POSITIVE.
    Typically, if the notation is \odot, it is the Hamiltonian
    as opposed to the Shuster.

    Parameters
    ----------
    a_quat
    b_quat
    scalar_last:
        Treat input and output quaternions in scalar-last notation
    """

    if scalar_last:
        # convert scalar last to scalar first
        a_quat = np.roll(a_quat, 1)
        b_quat = np.roll(b_quat, 1)

    ab_vector = (
        (a_quat[0] * b_quat[1:])
        + (b_quat[0] * a_quat[1:])
        + np.cross(a_quat[1:], b_quat[1:])
    )
    ab = np.array(
        [
            a_quat[0] * b_quat[0] - np.dot(a_quat[1:], b_quat[1:]),
            ab_vector[0],
            ab_vector[1],
            ab_vector[2],
        ]
    )

    # if scalar is supposed to be last,
    # roll the scalar at the front towards the end
    return ab if not scalar_last else np.array(ab, -1)


def shuster(a_quat, b_quat, scalar_last=False):
    """Compute the shuster product of two quaternions.

    Note the cross product of the vector component is NEGATIVE.
    Typically, if the notation is \otimes, it is shuster.

    """

    if scalar_last:
        # convert scalar last to scalar first
        a_quat = np.roll(a_quat, 1)
        b_quat = np.roll(b_quat, 1)

    ab_vector = (a_quat[0] * b_quat[1:] + b_quat[0] * a_quat[1:]) - np.cross(
        a_quat[1:], b_quat[1:]
    )
    ab = np.array(
        [
            a_quat[0] * b_quat[0] - np.dot(a_quat[1:], b_quat[1:]),
            ab_vector[0],
            ab_vector[1],
            ab_vector[2],
        ]
    )

    # if scalar is supposed to be last,
    # roll the scalar at the front towards the end
    return ab if not scalar_last else np.roll(ab, -1)


def h_sandwich(quat, vect, scalar_last=False):
    # if the quaternion is scalar last,
    # change to scalar first and proceed
    if scalar_last:
        quat = np.roll(quat, 1)

    quat_v = np.array([0, vect[0], vect[1], vect[2]])
    result = hamiltonian(hamiltonian(quat, quat_v), conjugate(quat))
    return result[1:]


def s_sandwich(quat, vect, scalar_last=False):
    # if the quaternion is scalar last,
    # change to scalar first and proceed
    if scalar_last:
        quat = np.roll(quat, 1)

    quat_v = np.array([0, vect[0], vect[1], vect[2]])
    result = shuster(shuster(quat, quat_v), conjugate(quat))

    return result[1:]
