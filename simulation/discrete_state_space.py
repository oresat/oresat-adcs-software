import numpy as np
#from olaf import logger
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
import logging
logger = logging.getLogger(__name__)

def add_integrators(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """Create the augmented A matrix for LQR design with an integrator.

    Parameters
    ----------
    a: np.ndarray
        Original system matrix (n x n)
    b: np.ndarray
        Input matrix (n x m)
    c: np.ndarray
        Output matrix (m x n)

    Returns
    -------
    numpy.ndarray
        Augmented A matrix [(n+m) x (n+m)]
    """
    # Get dimensions
    n = a.shape[0]  # Number of states
    m = c.shape[0]  # Number of outputs

    # Verify dimensions
    if a.shape != (n, n):
        raise ValueError("Matrix A must be square")
    if c.shape[1] != n:
        raise ValueError("Number of columns in C must match number of rows in A")

    # Create zero matrices
    zero_top_right = np.zeros((n, m))
    zero_bottom_right = np.zeros((m, m))

    # Construct augmented A matrix
    a_aug = np.block([[a, zero_top_right], [-c, zero_bottom_right]])

    # Construct augmented B matrix (always just added zeros to bottom of vector)
    b_zeros = np.zeros([m, b.shape[1]])
    b_aug = np.block([[b], [b_zeros]])
    return a_aug, b_aug


def get_gain_matrix(
    j: np.ndarray,
    timestep: float,
    max_error: float,
    max_rate: float,
    max_input: float,
    use_integrator: bool = False,
) -> np.ndarray:
    """Compute a gain matrix

    Parameters
    ----------
    j : np.ndarray
        Satellite inertia matrix
    timestep : float
        The update rate of the ADCS
    max_error : float
        LQR maximum error
    max_rate : float
        LQR maximum rate
    max_input : float
        The maximum acceptable value for control inputs (maximum torque)
    use_integrator : bool
        If True, use the integrator
    Returns
    -------
    np.ndarray
        The gain matrix
    """
    # LQR matrices
    max_error = max_error  # q_vec error
    max_velocity = max_rate  # ω_sat
    # integrator term in Q matrix, integrator state, accumulated error
    # (shouldn't exceed Q values for quaternion error)
    max_integrator = 0.1

    q = np.diag(
        [
            1 / max_error**2,
            1 / max_error**2,
            1 / max_error**2,
            1 / max_velocity**2,
            1 / max_velocity**2,
            1 / max_velocity**2,
            1 / max_integrator**2,
            1 / max_integrator**2,
            1 / max_integrator**2,
        ]
    )
    r = np.diag([1 / max_input**2, 1 / max_input**2, 1 / max_input**2])

    """
    When using the left-error quaternion convention, meaning q_error = q_target * q_current^-1,
    the derivative of the error quaternion is negative, so A becomes negative.
    Currently using the right-handed convention so A is positive.
    """
    # A matrix: maps ω into q_dot, ω_dot is driven by control input (J^{-1} u)
    a = 0.5 * np.eye(6, 6, 3)
    b = np.block([[np.zeros((3, 3))], [np.linalg.inv(j)]])
    # sensors for all inputs
    c = np.identity(6)
    # integrator only cares about attitude error, only integrate quaternion values (top half of
    # C matrix)
    c_aug = np.eye(3, 6)

    if use_integrator:
        a_aug, b_aug = add_integrators(a, b, c_aug)
        d_aug = np.zeros((c_aug.shape[0], b_aug.shape[1]))

        ad, bd, _cd, _dd, _dt = cont2discrete((a_aug, b_aug, c_aug, d_aug), timestep)
        p = solve_discrete_are(ad, bd, q, r)
        k = np.linalg.inv(r + bd.T @ p @ bd) @ bd.T @ p @ ad
    else:
        d = np.zeros((c.shape[0], b.shape[1]))

        ad, bd, _cd, _dd, _dt = cont2discrete((a, b, c, d), timestep)
        p = solve_discrete_are(ad, bd, q[:6, :6], r)
        k = np.linalg.inv(r + bd.T @ p @ bd) @ bd.T @ p @ ad

    a_cl = ad - bd @ k  # Discrete closed-loop matrix
    eigenvalues = np.linalg.eigvals(a_cl)

    for i, eig in enumerate(eigenvalues):
        if abs(eig) > 1:
            logger.warning("Eigenvalue outside of unit circle")

    return k
