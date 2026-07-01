from dataclasses import dataclass, field
from typing import Optional, Tuple

from datetime import datetime
import numpy as np
#from olaf import logger
from skyfield.framelib import itrs
from skyfield.timelib import Timescale

import logging
logger = logging.getLogger(__name__)

import quaternion as quat
from config import GuidanceMode

D2R = np.pi / 180.0


def gps_to_ecef(lat: float, lon: float, height: float) -> np.ndarray:
    # constants used for GPS-to-ECEF conversion
    a = 6378137.0  # WGS-84 constant: a = semi-major axis
    e2 = 0.0066943799901413165  # WGS-84 constant: e^2 = flattening

    sin_lat = np.sin(lat * D2R)
    cos_lat = np.cos(lat * D2R)

    n = a / np.sqrt(1 - e2 * sin_lat**2)  # latitude must be signed for WGS-84
    x = (n + height) * cos_lat * np.cos(lon * D2R)
    y = (n + height) * cos_lat * np.sin(lon * D2R)
    z = (n * (1 - e2) + height) * sin_lat

    return np.asarray([x, y, z])


@dataclass
class GroundStation:
    name: str
    lat: float
    lon: float
    height: int
    ecef: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.ecef = gps_to_ecef(self.lat, self.lon, self.height)


station_list = [
    # GroundStation("ESI", 39.608251, -104.895788, 1716),
    GroundStation("KSAT_Svalbard", 78.231500, 15.411100, 488),
    GroundStation("Deadhorse, AK", 70.201, -148.46, 0),
    GroundStation("Tampere", 61.497, 23.761, 0),
    GroundStation("Kaspichan", 43.31, 27.15, 0),
    GroundStation("Azores (Santa Maria)", 36.973, -25.17, 0),
    GroundStation("Columbus, OH", 39.961, -82.999, 0),
    GroundStation("Jeju", 33.50, 126.52, 0),
    GroundStation("Pretoria", -25.86, 28.45, 0),
    GroundStation("Pitea", 65.34, 21.42, 0),
]


def julian_date(iso_date:str) -> float:
    '''Calculate the julian date.

    Does NOT account for leap second.

    iso_timestamp
        ISO formatted timestamp in UTC.
    '''
    dt = datetime.fromisoformat(iso_date)
    term1 = int(7/4 * (dt.year + int((dt.month + 9) / 12)))
    term2 = int(275*dt.month / 9)
    term3 = (60*dt.hour + dt.minute + dt.second/(60)) / 1440
    return 1721013.5 + 367*dt.year + dt.day - term1 + term2 + term3

def sun_vector(julian_date, position_ecef, eci_2_ecef):
    '''Unit vector in inertial coordinates pointing from satellite to the sun.

    More details on page 420 of Markely & Crassidis. For now assume the sun is a constant distance away.

    Parameters
    ----------
    julian_date
    position_ecef
    '''
    # time in years
    T_UT1      = (julian_date - 2451545) / 36525
    # mean longitude (degrees)
    mean_long  = (280.46 + 36000.771 * T_UT1) % 360
    # mean anomaly (radians)
    mean_anom  = np.radians((357.5277233 + 35999.05034 * T_UT1) % 360)
    # ecliptic longitude
    ecl_long   = np.radians(mean_long + 1.914666471 * np.sin(mean_anom) + 0.019994643 * np.sin(2*mean_anom))
    # ecliptic oblique
    ecl_oblq   = np.radians(23.439291 - 0.0130042 * T_UT1)

    # unit vector of earth to sun, GCI coordinate frame
    earth_to_sun = np.array([np.cos(ecl_long),
                            np.cos(ecl_oblq) * np.sin(ecl_long),
                            np.sin(ecl_oblq) * np.sin(ecl_long)])


    position_eci = eci_2_ecef.T @ (position_ecef / np.linalg.norm(position_ecef))
    # GCI coordinate frame
    # mutliply by AU (meters) to get distance in meters
    sat_to_sun   = 1.496e11 * earth_to_sun - position_eci
    return sat_to_sun / np.linalg.norm(sat_to_sun)


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

    c_bn = np.vstack((xvec, yvec, zvec))  # Create DCM for body orientation in ECI coordinates

    target_quat = quat.quat_from_dcm_scalar_last(c_bn)  # Convert DCM to quaternion
    return target_quat


def sun_quat(sun_vector_eci, nadir_vector_ecef, eci_2_ecef) -> np.ndarray:
    """Finds the nearest sun_vector quaternion relative to ref_quaternion.

    Assumes scalar last notation.

    sun_vector_eci
        sun vector in eci coordinates
    nadir_vector_ecef
        nadir vector in ecef coordinates
    quat_eci
        quaternion of body from to ECI coordinates
    """
    # calculate the vector in ECI coordinates
    ecef_2_eci = eci_2_ecef.T

    # make sure it is a unit vector
    sun_vector_eci = sun_vector_eci / np.linalg.norm(sun_vector_eci)
    nadir_vector_eci = ecef_2_eci @ (nadir_vector_ecef / np.linalg.norm(nadir_vector_ecef))

    # primary objective
    # I want +y to face sun (solar panel)
    yvec = sun_vector_eci
    yvec = yvec / np.linalg.norm(yvec)

    # secondary objective
    # I want +x (star tracker) to be away from earth and sun
    # technically, the opposite direction of this is also valid
    # also, ensure it is orthogonal to the proper axes!
    # hold on this is going to flip all the time...
    xvec = np.cross(sun_vector_eci, nadir_vector_eci)
    xvec = xvec / np.linalg.norm(xvec)

    zvec = np.cross(xvec, yvec)
    zvec = zvec / np.linalg.norm(zvec)

    # in some instances where it cannot be guaranteed that
    # the secondary objective is not orthogonal to the primary objective,
    # the cross product can be taken one more time
    xvec = np.cross(yvec, zvec)
    xvec = xvec / np.linalg.norm(xvec)

    c_bn = np.vstack((xvec, yvec, zvec))  # Create DCM for body orientation in ECI coordinates

    target_quat = quat.quat_from_dcm_scalar_last(c_bn)
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

    c_bn = np.vstack((xvec, yvec, zvec))  # Create DCM for body orientation in ECI coordinates

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
            logger.error("unknown drag orientation {}. Defaulting to MIN_DRAG", drag_orientation)
        # flip vector such that in min_drag mode
        # the satellite's solar panels (rather than the GPS antenna) are pointing anti-nadir
        nadir_facing = -nadir_facing
        yvec = np.cross(drag_facing, nadir_facing)
        yvec = yvec / np.linalg.norm(yvec)

        # Create DCM for body orientation in ECI coordinates
        c_bn = np.vstack((nadir_facing, yvec, drag_facing))

    target_quat = quat.quat_from_dcm_scalar_last(c_bn)  # Convert DCM to quaternion
    return target_quat


def psi_c2_c3(chi: float, alpha: float) -> Tuple[float, float, float]:
    psi = alpha * chi**2
    if abs(psi) < 1e-8:  # Use series near 0 for numerical stability
        c2 = 0.5 - psi / 24.0 + psi**2 / 720.0 - psi**3 / 40320.0
        c3 = 1.0 / 6.0 - psi / 120.0 + psi**2 / 5040.0 - psi**3 / 362880.0
    elif psi > 0:
        s = np.sqrt(psi)
        c2 = (1.0 - np.cos(s)) / psi
        c3 = (s - np.sin(s)) / (s**3)
    else:
        s = np.sqrt(-psi)
        c2 = (np.cosh(s) - 1.0) / (-psi)
        c3 = (np.sinh(s) - s) / (s**3)

    return psi, c2, c3


def kepler_values(r0_vec: np.ndarray, v0_vec: np.ndarray, dt: int) -> np.ndarray:
    """Kepler propagation function as defined by Mathworks:
    https://www.mathworks.com/help/aerotbx/ug/orbit-pop-algorithms.html
    """

    mu_earth = 3.986e14
    sqrt_mu = np.sqrt(mu_earth)

    r0 = np.linalg.norm(r0_vec)
    v0 = np.linalg.norm(v0_vec)
    vr0 = np.dot(r0_vec, v0_vec) / r0  # Radial velocity component v_r0 = (r0·v0)/|r0|

    epsilon = v0**2 / 2 - mu_earth / r0  # determine orbital energy
    alpha = -2 * epsilon / mu_earth  # alpha should always be > 0 for an elliptical orbit
    chi = sqrt_mu * dt * alpha  # initial guess for first iteration
    psi, c2, c3 = psi_c2_c3(chi, alpha)

    iterations = 0
    max_iter = 100000
    while True:
        chi_new = chi + (
            sqrt_mu * dt
            - chi**3 * c3
            - r0 * vr0 * chi**2 * c2 / sqrt_mu
            - r0 * chi * (1 - psi * c3)
        ) / (chi**2 * c2 + r0 * vr0 * chi * (1 - psi * c3) / sqrt_mu + r0 * (1 - psi * c2))
        psi, c2, c3 = psi_c2_c3(chi_new, alpha)

        if abs(chi_new - chi) < 1e-8:
            chi = chi_new
            break
        chi = chi_new  # update Chi and iterate again

        iterations += 1
        if iterations > max_iter:
            logger.debug("Maximum iterations reached in Kepler Propagation. Returning")
            return None

    # calculate universal variables
    f = 1 - chi**2 / r0 * c2
    g = dt - chi**3 / sqrt_mu * c3

    r = f * np.asarray(r0_vec) + g * np.asarray(v0_vec)

    return r


def R3(theta: float) -> np.ndarray:
    """Used for Earth rotation calculations. Not a perfect model for Earth's rotation,
    but fast and good enough for overpass window determination.
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def time_to_overpass(
    skyfield_timescale: Timescale,
    time_range_hours: int,
    max_distance: int,
    r_ecef: np.ndarray,
    v_ecef: np.ndarray,
    target_ecef: np.ndarray,
):
    """
    A function to determine how long the spacecraft can enter low-power mode
    before it will be within range of a set of GPS coordinates for fine-pointing
    of antenna and transmission

    Parameters
    ----------
    fsw_obj : object
        The flight software object
    time_range_hours : int
        time in hours up to which overpass possibility should be checked
    max_distance : int
        maximum distance from target, in meters
    r_ecef : np.ndarray
        position relative to Earth in ECEF
    v_ecef : np.ndarray
        velocity relative to Earth in ECEF
    target_ecef : np.ndarray
        Target coordinates
    """
    omega_earth = 7.2921150e-5
    large_dt = 60 * 5  # for coarse overpass determination (five minutes)
    small_dt = 5  # for fine overpass determination
    maximum_window = 3600  # large value to ensure exit boundary of overpass window is found
    exit_check_increment = 5  # search for window exit in increments
    # lead time before entering window to engage control system. Lets satellite reorient in time.
    lead_time = 100
    min_window_time = 300  # minimum overpass time
    # time to skip after window with insufficient overpass time before searching again
    skip_after_window = 2000

    # Is this supposed to be the time when the filter was init'd or the current time?
    time = skyfield_timescale.now()

    # itrs.rotation_at(t) returns the rotation matrix that maps ICRF/ECI -> ITRS/ECEF.
    r_ecef_from_eci = itrs.rotation_at(time)
    r_eci_from_ecef = r_ecef_from_eci.T  # get current Earth rotation angle relative to ECI
    # Get Earth's rotation angle in ECI at current time. Radians in [-pi, pi]
    theta0 = np.arctan2(r_eci_from_ecef[1, 0], r_eci_from_ecef[0, 0])

    # convert orbital vector parameters from ECEF to ECI
    r_eci = r_eci_from_ecef @ r_ecef
    v_eci = r_eci_from_ecef @ v_ecef

    def distance_to_target_eci(dt) -> float:
        r_eci_new = kepler_values(r_eci, v_eci, dt)
        theta = theta0 + dt * omega_earth
        target_eci = R3(theta) @ target_ecef
        return np.linalg.norm(r_eci_new - target_eci)

    time_offset = 0  # window search offset
    t_end = time_range_hours * 3600  # convert hours to seconds
    # scan up until maximum time for a window which satisfies minimum time criteria
    while time_offset < t_end:
        # first coarse scan to find window
        overpass_time: Optional[int] = None
        # check all future positions in propagation intervals
        for dt in range(time_offset, t_end, large_dt):
            if distance_to_target_eci(dt) <= max_distance:
                overpass_time = dt  # we are passing over within range at this time
                break

        if overpass_time is None:  # no overpass found within window
            logger.error(
                "Overpass window not found in {} hour time window. Check if inclination allows for"
                "overpass within {} km",
                time_range_hours,
                max_distance / 1e3,
            )
            return 0, None

        # second scan with finer interval to narrow down window-entry time
        # wind clock back by one time interval in order to scan in finer intervals
        lower_bound = max(overpass_time - large_dt, time_offset)
        upper_bound = overpass_time + 1
        window_start: int = 0
        window_exit: int = 0

        # check all future positions in finer propagation intervals
        for dt in range(lower_bound, upper_bound, small_dt):
            if distance_to_target_eci(dt) <= max_distance:
                window_start = dt  # save time at which SC is in range
                logger.info(f"Overpass window entry predicted to occur in {window_start} seconds")
                break

        # third step: find window exit time
        for dt in range(window_start, window_start + maximum_window, exit_check_increment):
            if distance_to_target_eci(dt) > max_distance:
                window_exit = dt  # window exit at this time
                logger.info(f"Overpass window exit predicted to occur in {window_exit} seconds")
                break

        # if an acceptable window has been found, return value
        if (window_exit - window_start) >= min_window_time:
            # ensure controller can't see negative activation time
            # (would work anyway in current implementation, but this avoids future bugs)
            controller_start = max(0, window_start - lead_time)
            controller_end = window_exit
            return controller_start, controller_end

        # Too short: advance offset and try again
        time_offset = window_exit + skip_after_window

    logger.error(
        "Sufficiently large overpass window not found in specified time window of {} hours. Check"
        "if inclination allows for overpass within {} km",
        time_range_hours,
        max_distance / 1e3,
    )
    return 0, None


def find_nearest_ground_station(
    skyfield_timescale: Timescale,
    time_range_hours: int,
    max_distance: int,
    r_ecef: np.ndarray,
    v_ecef: np.ndarray,
) -> Tuple[Optional[int], Optional[int]]:
    """Find the nearest ground station and return the overpass window times

    Parameters
    ----------
    fsw_obj : object
        The flight software object
    time_range_hours : int
        time in hours up to which overpass possibility should be checked
    max_distance : int
        maximum distance from target, in meters
    r_ecef : np.ndarray
        position relative to Earth in ECEF
    v_ecef : np.ndarray
        velocity relative to Earth in ECEF


    Returns
    -------

    """
    chosen_station: Optional[GroundStation] = None
    next_overpass: list = None  # seconds
    for station in station_list:
        logger.debug(f"Scanning {station.name}")
        start, end = time_to_overpass(
            skyfield_timescale, time_range_hours, max_distance, r_ecef, v_ecef, station.ecef
        )
        if start == -1 or start == -2:
            start = None  # deal with error messages
        if (start is not None) and ((next_overpass is None) or (start < next_overpass[0])):
            chosen_station = station
            next_overpass = [start, end]

    if chosen_station:
        logger.info(f"Next overpass opportunity is {chosen_station.name}")
        logger.info(f"Overpass window entry predicted to occur in {next_overpass[0]} seconds")
        logger.info(f"Overpass window exit predicted to occur in {next_overpass[1]} seconds")
        return next_overpass[0], next_overpass[1]
    else:
        logger.debug("No overpasses found within range")
        return None, None
