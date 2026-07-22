from dataclasses import dataclass, field
from typing import Optional, Tuple

from datetime import datetime
import numpy as np
# from olaf import logger

import quaternion as quat
from config import GuidanceMode


import logging

logger = logging.getLogger(__name__)


def julian_date(iso_date: str) -> float:
    """Calculate the julian date.

    Does NOT account for leap second.

    iso_timestamp
        ISO formatted timestamp in UTC.
    """
    dt = datetime.fromisoformat(iso_date)
    term1 = int(7 / 4 * (dt.year + int((dt.month + 9) / 12)))
    term2 = int(275 * dt.month / 9)
    term3 = (60 * dt.hour + dt.minute + dt.second / (60)) / 1440
    return 1721013.5 + 367 * dt.year + dt.day - term1 + term2 + term3


def sun_vector(julian_date, position_ecef, eci_2_ecef):
    """Unit vector in inertial coordinates pointing from satellite to the sun.

    More details on page 420 of Markely & Crassidis. For now assume the sun is a constant distance away.

    Parameters
    ----------
    julian_date
    position_ecef
    """
    # time in years
    T_UT1 = (julian_date - 2451545) / 36525
    # mean longitude (degrees)
    mean_long = (280.46 + 36000.771 * T_UT1) % 360
    # mean anomaly (radians)
    mean_anom = np.radians((357.5277233 + 35999.05034 * T_UT1) % 360)
    # ecliptic longitude
    ecl_long = np.radians(
        mean_long
        + 1.914666471 * np.sin(mean_anom)
        + 0.019994643 * np.sin(2 * mean_anom)
    )
    # ecliptic oblique
    ecl_oblq = np.radians(23.439291 - 0.0130042 * T_UT1)

    # unit vector of earth to sun, GCI coordinate frame
    earth_to_sun = np.array(
        [
            np.cos(ecl_long),
            np.cos(ecl_oblq) * np.sin(ecl_long),
            np.sin(ecl_oblq) * np.sin(ecl_long),
        ]
    )

    position_eci = eci_2_ecef.T @ (position_ecef / np.linalg.norm(position_ecef))
    # GCI coordinate frame
    # mutliply by AU (meters) to get distance in meters
    sat_to_sun = 1.496e11 * earth_to_sun - position_eci
    return sat_to_sun / np.linalg.norm(sat_to_sun)


def orthonormalize_vectors(a_vect, b_vect):
    """Make an orthonormal set of two vectors.

    See Gram-Schmidt algorithm."""
    # normalize a
    a_vect = a_vect / np.linalg.norm(a_vect)
    # remove (projection of b on a) from b
    b_vect = b_vect - b_vect * np.dot(a_vect, b_vect) / np.dot(b_vect, b_vect)
    # normalize b
    b_vect = b_vect / np.linalg.norm(b_vect)

    return a_vect, b_vect


def vector_transform_quat(a_vect, b_vect, scalar_last=False):
    """Calculate the quaternion to rotate vector a to vector b"""

    # normalize vectors
    a_vect = a_vect / np.linalg.norm(a_vect)
    b_vect = b_vect / np.linalg.norm(b_vect)
    # calculate the cross product for the axis of rotation
    axis = np.cross(a_vect, b_vect)
    axis = axis / np.linalg.norm(axis)
    radians = np.arccos(np.dot(a_vect, b_vect))
    scalar = np.array(np.cos(radians / 2))
    vector = np.array(axis * np.sin(radians / 2))
    result_quat = np.array([
        scalar,
        vector[0],
        vector[1],
        vector[2]
        ])

    return result_quat if not scalar_last else np.roll(result_quat, -1)


def orthonormal_transform_quat(a_vect_1, b_vect_1, a_vect_2, b_vect_2):
    """Find quaternion that will transform one orthonormal set to another.

    Parameters
    a_1
        First vector in starting orthonormal set (body frame)
    b_1
        First vector in destination orthonormal set (inertial frame)
    a_2
        Second vector in starting orthonormal set (body frame)
    b_2
        Second vector in destination orthonormal set (inertial frame)
    """
    # get the quaternion for the first rotation
    eci_quat_1 = vector_transform_quat(a_vect_1, b_vect_1)
    middle_vect_2 = quat.h_sandwich(eci_quat_1, a_vect_2)
    eci_quat_2 = vector_transform_quat(middle_vect_2, b_vect_2)

    # combine two quats in eci refrence frame
    return quat.hamiltonian(eci_quat_2, eci_quat_1)


def one_boresight_quat(instrument_1_body, target_1_ecef, eci_2_ecef):
    """Finds the eci quaternion with the smallest rotation to get instrument 1 to point towards target 1"""
    # let me normalize this for you
    target_1_eci = eci_2_ecef.T @ (target_1_ecef / np.linalg.norm(target_1_ecef))

    return vector_transform_quat(instrument_1_body, target_1_eci)

def two_boresights_quat(boresight_1, target_1, boresight_2, target_2, scalar_last=False):
    """Find quaternion transformation, prioritizing one boresight-target pair over another.

    The higher priority boresight-target pair will attempt to be aligned.
    The lower priority boresight-target pair will have its angle minimized, contrained by the higher priority.
    This is done by making the high-low priority pairs orthonormal pairs, where the direction
    of the lower-priority vector is adjusted to be normal to the higher priority vector.

    Parameters
    ----------
    boresight_1
        vector in body frame with higher priority (for zero error)
    target_1
        vector in inertial frame with lower priority (for zero error)
    boresight_2
        vector in body frame with lower priority (minimize error)
    target_2
        vector in inertial frame with lower priority (minimize error)"""
    # make orthonormal pairs
    b_1, b_2 = orthonormalize_vectors(boresight_1, boresight_2)
    t_1, t_2 = orthonormalize_vectors(target_1, target_2)
    # find the transformation from one pair to the other
    result_quat = orthonormal_transform_quat(b_1, t_1, b_2, t_2)

    return result_quat if not scalar_last else np.roll(result_quat, -1)

def sun_quat(
    sun_vector_eci, nadir_vector_ecef, velocity_vector_ecef, eci_2_ecef
) -> np.ndarray:
    """Finds the nearest sun_vector quaternion relative to ref_quaternion.

    Assumes scalar last notation.

    sun_vector_eci
        sun vector in eci coordinates
    nadir_vector_ecef
        nadir vector in ecef coordinates
    vect_2_body
        the vector of the secondary objective in body frame
    quat
        A quaternion in eci frame
    eci_2_ecef
        quaternion of body from to ECI coordinates
    """
    # calculate the vector in ECI coordinates
    ecef_2_eci = eci_2_ecef.T

    # make sure it is a unit vector
    sun_vector_eci = sun_vector_eci / np.linalg.norm(sun_vector_eci)
    nadir_vector_eci = ecef_2_eci @ (
        nadir_vector_ecef / np.linalg.norm(nadir_vector_ecef)
    )
    velocity_vector_eci = ecef_2_eci @ (
        velocity_vector_ecef / np.linalg.norm(velocity_vector_ecef)
    )

    # primary objective
    # I want +y to face sun (solar panel)
    #yvec = sun_vector_eci
    #yvec = yvec / np.linalg.norm(yvec)
    #xvec = sun_vector_eci
    #xvec = xvec / np.linalg.norm(xvec)

    zvec = sun_vector_eci
    zvec = zvec / np.linalg.norm(zvec)



    # secondary objective
    # I want +x (star tracker) to be away from earth and sun
    # technically, the opposite direction of this is also valid
    # also, ensure it is orthogonal to the proper axes!
    # find the current eci vector of the +x-axis
    star_vector_eci = np.cross(nadir_vector_eci, velocity_vector_eci)
    star_vector_eci = star_vector_eci / np.linalg.norm(star_vector_eci)
    if np.dot(sun_vector_eci, star_vector_eci) < 0:
        star_vector_eci = -star_vector_eci

    # calculate where the x axis is pointing in eci
    # if the x-axis is already closer to the star axis vector,
    # go for it otherwise flip it.
    xvec = star_vector_eci
    xvec = xvec / np.linalg.norm(xvec)
    #yvec = star_vector_eci
    #yvec = yvec / np.linalg.norm(yvec)

    #zvec = np.cross(xvec, yvec)
    #zvec = zvec / np.linalg.norm(zvec)

    yvec = np.cross(zvec, xvec)
    yvec = yvec / np.linalg.norm(yvec)


    # in some instances where it cannot be guaranteed that
    # the secondary objective is not orthogonal to the primary objective,
    # the cross product can be taken one more time
    xvec = np.cross(yvec, zvec)
    #yvec = np.cross(zvec, xvec)

    c_bn = np.vstack(
        (xvec, yvec, zvec)
    )  # Create DCM for body orientation in ECI coordinates

    target_quat = quat.quat_from_dcm_scalar_last(c_bn)

    # try my way
    target_quat = vector_transform_quat(
        np.array([-1, 1, 0]),
        sun_vector_eci,
        scalar_last = True
    )

    #target_quat = two_boresights_quat(
    #    boresight_1 = np.array([0, 1, 0]), # have two solar panel sides both be exposed towards sun
    #    target_1 = sun_vector_eci,
    #    boresight_2 = np.array([1, 0, 0]),  # have star tracker point away towards stars
    #    target_2 = star_vector_eci,
    #    scalar_last = True
    #)
    return target_quat


def target_tracking_quat(
    target_vector: np.ndarray, nadir_vector_ecef: np.ndarray, eci_2_ecef: np.ndarray
) -> np.ndarray:
    """Creates an orientation quaternion forming an orientation based on a target
    vector for the z-facing, and orients the +x facing to point into the starfield
    (away from nadir vector) in order to give star tracker unoccluded view
    """
    r_ne = eci_2_ecef.T  # rotation matrix from ECEF to ECI

    # norm target vector and convert to ECI
    zvec = r_ne @ (target_vector / np.linalg.norm(target_vector))

    neg_nadir_eci = r_ne @ (-nadir_vector_ecef)
    # remove component parallel to nadir vector from velocity vector
    # to determine "ram-facing-like" vector
    xvec = neg_nadir_eci - np.dot(neg_nadir_eci, zvec) * zvec
    xvec = xvec / np.linalg.norm(xvec)

    yvec = np.cross(zvec, xvec)
    yvec = yvec / np.linalg.norm(yvec)

    c_bn = np.vstack(
        (xvec, yvec, zvec)
    )  # Create DCM for body orientation in ECI coordinates

    target_quat = quat.quat_from_dcm_scalar_last(c_bn)  # Convert DCM to quaternion
    return target_quat


def nadir_quat(
    nadir_vector_ecef: np.ndarray, v_ecef: np.ndarray, eci_2_ecef: np.ndarray
) -> np.ndarray:
    """
    Creates an orientation quaternion forming an orientation based on a nadir
    vector for the z-facing, and orients the +x facing towards the velocity vector
    """
    r_ne = eci_2_ecef.T  # rotation matrix from ECEF to ECI
    # norm velocity vector and convert to ECI
    v_eci = r_ne @ (v_ecef / np.linalg.norm(v_ecef))

    # norm target vector and convert to ECI
    zvec = r_ne @ (nadir_vector_ecef / np.linalg.norm(nadir_vector_ecef))

    # remove component parallel to nadir vector from velocity vector
    # to determine "ram-facing-like" vector
    xvec = v_eci - np.dot(v_eci, zvec) * zvec
    xvec = xvec / np.linalg.norm(xvec)

    yvec = np.cross(zvec, xvec)
    yvec = yvec / np.linalg.norm(yvec)

    c_bn = np.vstack(
        (xvec, yvec, zvec)
    )  # Create DCM for body orientation in ECI coordinates

    target_quat = quat.quat_from_dcm_scalar_last(c_bn)  # Convert DCM to quaternion
    return target_quat


def ram_quaternion(
    drag_orientation: GuidanceMode,
    v_ecef: np.ndarray,
    nadir_vector_ecef: np.ndarray,
    eci_2_ecef: np.ndarray,
) -> np.ndarray:
    """
    Creates an orientation quaternion forming based on whether maximum or
    minimum drag is desired. The secondary axis is defined as the nadir
    vector, or as close as possible to it
    """
    r_ne = eci_2_ecef.T
    # norm velocity vector and convert to ECI
    drag_facing = r_ne @ (v_ecef / np.linalg.norm(v_ecef))

    nadir_eci = r_ne @ nadir_vector_ecef
    # remove component parallel to velocity vector from nadir vector to determine
    # "downwards-pointing" vector
    nadir_facing = nadir_eci - np.dot(nadir_eci, drag_facing) * drag_facing
    nadir_facing = nadir_facing / np.linalg.norm(nadir_facing)

    if drag_orientation == GuidanceMode.MAX_DRAG:
        yvec = np.cross(nadir_facing, drag_facing)
        yvec = yvec / np.linalg.norm(yvec)

        # Create DCM for body orientation in ECI coordinates
        c_bn = np.vstack((drag_facing, yvec, nadir_facing))
    else:
        if drag_orientation != GuidanceMode.MIN_DRAG:
            logger.error(
                "unknown drag orientation {}. Defaulting to MIN_DRAG", drag_orientation
            )
        # flip vector such that in min_drag mode
        # the satellite's solar panels (rather than the GPS antenna) are pointing anti-nadir
        nadir_facing = -nadir_facing
        yvec = np.cross(drag_facing, nadir_facing)
        yvec = yvec / np.linalg.norm(yvec)

        # Create DCM for body orientation in ECI coordinates
        c_bn = np.vstack((nadir_facing, yvec, drag_facing))

    target_quat = quat.quat_from_dcm_scalar_last(c_bn)  # Convert DCM to quaternion
    return target_quat
