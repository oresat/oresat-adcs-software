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


def orthonormalize_vectors(vect_1, vect_2):
    """Make an orthonormal set of two vectors (see Gram-Schmidt algorithm).

    Parameters
    ----------
    vect_1
        First vector which is only normalized
    vect_2
        Second vector which is made orthonormal to first vector
    """
    # normalize first vector
    vect_1 = vect_1 / np.linalg.norm(vect_1)
    # make orthogonal: remove (projection of vect_2 on vect_1) from vect_2
    vect_2 = vect_2 - vect_1 * np.dot(vect_1, vect_2) / np.dot(vect_2, vect_2)
    # normalize vect_2
    vect_2 = vect_2 / np.linalg.norm(vect_2)

    return vect_1, vect_2


def vector_transform_quat(normvect_1, normvect_2, scalar_last=False):
    """Calculate the quaternion to rotate normalized vector 1 to normalized vector 2.

    Parameters
    ----------
    normvect_1
        normalized vector 1
    normvect_2
        normalized vector 1
    scalar_last
        Put the scalar term of the quaternion last
    """
    # calculate the cross product for the axis of rotation
    axis = np.cross(normvect_1, normvect_2)
    axis = axis / np.linalg.norm(axis)
    radians = np.arccos(np.dot(normvect_1, normvect_2))
    result_quat = np.array(
        [
            np.cos(radians / 2),
            axis[0] * np.sin(radians / 2),
            axis[1] * np.sin(radians / 2),
            axis[2] * np.sin(radians / 2),
        ]
    )

    return result_quat if not scalar_last else np.roll(result_quat, -1)


def one_boresight_quat(boresight_1, target_1, scalar_last=False):
    """Calculate the quaternion with the smallest rotation to align boresight with target.

    Parameters
    ----------
    boresight_1
        boresight vector in body coordinates, does not need to be normalized first
    target_1
        target vector in inertial coordinates (ECI)
    scalar_last
        Put the scalar term of the quaternion last
    """
    b_1 = boresight_1 / np.linalg.norm(boresight_1)
    t_1 = target_1 / np.linalg.norm(target_1)
    result_quat = vector_transform_quat(normvect_1=b_1, normvect_2=t_1)

    return result_quat if not scalar_last else np.roll(result_quat, -1)


def two_boresights_quat(
    boresight_1, target_1, boresight_2, target_2, scalar_last=False
):
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
        vector in inertial frame with lower priority (minimize error)
    """
    # Make boresights and targets into orthonormal pairs.
    b_1, b_2 = orthonormalize_vectors(vect_1=boresight_1, vect_2=boresight_2)
    t_1, t_2 = orthonormalize_vectors(vect_1=target_1, vect_2=target_2)

    # Find the quaternion to match the orthonormal pairs.

    # Calculate the quaternion for the first boresight
    # to be aligned with the first target.
    quat_1 = vector_transform_quat(normvect_1=b_1, normvect_2=t_1)
    # Apply the quaternion to the second boresight to get an
    # intermediate boresight vector in inertial frame.
    middle_b_2 = quat.h_sandwich(quat=quat_1, vect=b_2)
    # Calculate the quaternion for the second boresight to be
    # aligned with second target.
    quat_2 = vector_transform_quat(normvect_1=middle_b_2, normvect_2=t_2)

    # Combine two quaternions into one quaternion for the intertial frame
    # note: the latter step is the first argument: quat_2(quat_1(vector))
    result_quat = quat.hamiltonian(quat_2, quat_1)

    return result_quat if not scalar_last else np.roll(result_quat, -1)




def sun_quat(
        sun_vector_eci: np.ndarray, nadir_vector_ecef: np.ndarray, velocity_vector_ecef: np.ndarray, eci_2_ecef: np.ndarray
) -> np.ndarray:
    """Finds the nearest sun_vector quaternion relative to ref_quaternion.

    Assumes scalar last notation.

    Parameters
    ----------
    sun_vector_eci
        sun vector in ECI coordinates
    nadir_vector_ecef
        nadir vector in ECEF coordinates
    velocity_vector_ecef
        velocity vector in ECEF coordinates
    eci_2_ecef
        ECI to ECEF conversion matrix
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

    # The star tracker should be pointed away from earth and sun.
    # This objective should not "flip" 180 degrees throughout the orbit.
    star_vector_eci = np.cross(nadir_vector_eci, velocity_vector_eci)
    star_vector_eci = star_vector_eci / np.linalg.norm(star_vector_eci)
    if np.dot(sun_vector_eci, star_vector_eci) < 0:
        star_vector_eci = -star_vector_eci

    # One option is to only have two solar panel sides both diagonally 
    # pointed towards the sun, regardless of star tracker,
    # in which you would call one_boresight_quat()

    # Another option is to include sun and nadir avoidance by having
    # the star tracker point towards the star field.
    target_quat = two_boresights_quat(
        boresight_1=np.array(
            [-1, 1, 0]
        ),  # have two solar panel sides both be exposed towards sun
        target_1=sun_vector_eci,
        boresight_2=np.array([1, 0, 0]),  # have star tracker point away towards stars
        target_2=star_vector_eci,
        scalar_last=True,
    )

    return target_quat


def target_tracking_quat(
    target_vector: np.ndarray, nadir_vector_ecef: np.ndarray, eci_2_ecef: np.ndarray
) -> np.ndarray:
    """Creates an orientation quaternion forming an orientation based on a target
    vector for the z-facing, and orients the +x facing to point into the starfield
    (away from nadir vector) in order to give star tracker unoccluded view
    """
    
    target_ecef = target_vector
    nadir_ecef = nadir_vector_ecef

    ecef_2_eci = eci_2_ecef.T

    # convert target vector to ECI coordinates
    target_eci = ecef_2_eci @ (target_ecef / np.linalg.norm(target_ecef))
    # temporarily define starfield as opposite of nadir
    star_eci = ecef_2_eci @ (- nadir_ecef / np.linalg.norm(nadir_ecef))

    target_quat = two_boresights_quat(
        boresight_1=np.array(
           [0, 0, 1]  # +z direction
        ),
        target_1=target_eci,
        boresight_2=np.array(
            [1, 0, 0]  # +x direction
        ),
        target_2=star_eci,
        scalar_last=True
    )

    return target_quat


def nadir_quat(
    nadir_ecef: np.ndarray, velocity_ecef: np.ndarray, eci_2_ecef: np.ndarray
) -> np.ndarray:
    """
    Creates an orientation quaternion forming an orientation based on a nadir
    vector for the z-facing, and orients the +x facing towards the velocity vector
    """
    # rotation matrix from ECEF to ECI
    ecef_2_eci = eci_2_ecef.T  

    # norm velocity vector and convert to ECI
    velocity_eci = ecef_2_eci @ (velocity_ecef / np.linalg.norm(velocity_ecef))

    # norm target vector and convert to ECI
    nadir_eci = ecef_2_eci @ (nadir_ecef / np.linalg.norm(nadir_ecef))

    target_quat = two_boresights_quat(
        boresight_1=np.array(
           [0, 0, 1]  # +z direction
        ),
        target_1=nadir_eci,
        boresight_2=np.array(
            [1, 0, 0]  # +x direction
        ),
        target_2=velocity_eci,
        scalar_last=True
    )

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
