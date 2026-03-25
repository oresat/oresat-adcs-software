import numpy as np
import Quaternions as quat
from Basilisk.utilities import macros
from skyfield.framelib import itrs

def GPS_to_ECEF(lat, lon, height):
    # constants used for GPS-to-ECEF conversion
    a = 6378137.0 # WGS-84 constant: a = semi-major axis
    e2 = 0.0066943799901413165 # WGS-84 constant: e^2 = flattening
    
    sin_lat = np.sin(lat * macros.D2R)
    cos_lat = np.cos(lat * macros.D2R)
    
    N = a/np.sqrt(1-e2*sin_lat**2) # lattitude must be signed for WGS-84
    x = (N+height)*cos_lat*np.cos(lon * macros.D2R)
    y = (N+height)*cos_lat*np.sin(lon * macros.D2R)
    z = (N*(1-e2)+height)*sin_lat

    return np.asarray([x, y, z])

class GroundStation:
    def __init__(self, name, lat, lon, height):
        self.name = name
        self.lat = lat
        self.lon = lon
        self.height = height
        self.ECEF = GPS_to_ECEF(lat, lon, height) # convert GPS input to ECEF coordinates

station_list = [
    # GroundStation("ESI", 39.608251, -104.895788, 1716),
    GroundStation("KSAT_Svalbard", 78.231500, 15.411100, 488), # ASK IF KSAT IS STILL VALID
    GroundStation("Deadhorse, AK", 70.201, -148.46, 0),
    GroundStation("Tampere", 61.497, 23.761, 0),
    GroundStation("Kaspichan", 43.31, 27.15, 0),
    GroundStation("Azores (Santa Maria)", 36.973, -25.17, 0),
    GroundStation("Columbus, OH", 39.961, -82.999, 0),
    GroundStation("Jeju", 33.50, 126.52, 0),
    GroundStation("Pretoria", -25.86, 28.45, 0),
    GroundStation("Pitea", 65.34, 21.42, 0),
]

def target_tracking_quat(target_vector, nadir_vector_ECEF, ECI_2_ECEF):
    '''
    Creates an orientation quaternion forming an orientation based on a target
    vector for the z-facing, and orients the +x facing to point into the starfield
    (away from nadir vector) in order to give star tracker unoccluded view
    '''
    R_NE = ECI_2_ECEF.T # rotation matrix from ECEF to ECI
    #OLD VERSION FOR +x RAM POINTING    
    # v_ECI = R_NE @ (v_ECEF/np.linalg.norm(v_ECEF)) # norm velocity vector and convert to ECI
        
    # zvec = R_NE @ (target_vector/np.linalg.norm(target_vector)) # norm target vector and convert to ECI
    
    # xvec = v_ECI - np.dot(v_ECI, zvec) * zvec # remove component parallel to nadir vector from velocity vector to determine "ram-facing-like" vector
    # xvec = xvec/np.linalg.norm(xvec) # norm
    

    zvec = R_NE @ (target_vector/np.linalg.norm(target_vector)) # norm target vector and convert to ECI for primary facing
    
    neg_nadir_ECI = R_NE @ (-nadir_vector_ECEF)
    xvec = neg_nadir_ECI - np.dot(neg_nadir_ECI, zvec) * zvec # remove component parallel to nadir vector from velocity vector to determine "ram-facing-like" vector
    xvec = xvec/np.linalg.norm(xvec) # norm

    yvec = np.cross(zvec, xvec)
    yvec = yvec/np.linalg.norm(yvec) # norm
    
    # xvec = np.cross(yvec, zvec) # Re-orthogonalize zB to avoid numerical drift
    # xvec = xvec/np.linalg.norm(xvec) # norm
    
    C_BN = np.vstack((xvec, yvec, zvec)) # Create DCM for body orientation in ECI coordinates
    
    target_quat = quat.quat_from_dcm_scalar_last(C_BN) # Convert DCM to quaternion
    return target_quat

def nadir_quat(nadir_vector_ECEF, v_ECEF, ECI_2_ECEF):
    '''
    Creates an orientation quaternion forming an orientation based on a nadir
    vector for the z-facing, and orients the +x facing towards the velocity vector
    '''
    
    R_NE = ECI_2_ECEF.T # rotation matrix from ECEF to ECI
    v_ECI = R_NE @ (v_ECEF/np.linalg.norm(v_ECEF)) # norm velocity vector and convert to ECI
    
    zvec = R_NE @ (nadir_vector_ECEF/np.linalg.norm(nadir_vector_ECEF)) # norm target vector and convert to ECI for primary facing
    
    xvec = v_ECI - np.dot(v_ECI, zvec) * zvec # remove component parallel to nadir vector from velocity vector to determine "ram-facing-like" vector
    xvec = xvec/np.linalg.norm(xvec) # norm

    yvec = np.cross(zvec, xvec)
    yvec = yvec/np.linalg.norm(yvec) # norm
    
    # xvec = np.cross(yvec, zvec) # Re-orthogonalize zB to avoid numerical drift
    # xvec = xvec/np.linalg.norm(xvec) # norm
    
    C_BN = np.vstack((xvec, yvec, zvec)) # Create DCM for body orientation in ECI coordinates
    
    target_quat = quat.quat_from_dcm_scalar_last(C_BN) # Convert DCM to quaternion
    return target_quat
    
def ram_quaternion(drag_orientation, v_ECEF, nadir_vector_ECEF, ECI_2_ECEF):
    '''
    Creates an orientation quaternion forming based on whether maximum or 
    minimum drag is desired. The secondary axis is defined as the nadir 
    vector, or as close as possible to it
    '''
    
    R_NE = ECI_2_ECEF.T
    drag_facing = R_NE @ (v_ECEF/np.linalg.norm(v_ECEF)) # norm velocity vector and convert to ECI
    
    nadir_ECI = R_NE @ nadir_vector_ECEF
    nadir_facing = nadir_ECI - np.dot(nadir_ECI, drag_facing) * drag_facing # remove component parallel to velocity vector from nadir vector to determine "downwards-pointing" vector
    nadir_facing = nadir_facing/np.linalg.norm(nadir_facing) # norm 
    
    if drag_orientation == "MAX_DRAG":
        yvec = np.cross(nadir_facing, drag_facing)
        yvec = yvec/np.linalg.norm(yvec) # norm
        
        C_BN = np.vstack((drag_facing, yvec, nadir_facing)) # Create DCM for body orientation in ECI coordinates
        
    elif drag_orientation == "MIN_DRAG":
        nadir_facing = -nadir_facing # flip vector such that in min_drag mode, the satellite's solar panels (rather than the GPS antenna) are pointing anti-nadir
        yvec = np.cross(drag_facing, nadir_facing)
        yvec = yvec/np.linalg.norm(yvec) # norm
        
        C_BN = np.vstack((nadir_facing, yvec, drag_facing)) # Create DCM for body orientation in ECI coordinates
    
    target_quat = quat.quat_from_dcm_scalar_last(C_BN) # Convert DCM to quaternion
    return target_quat
    
def psi_c2_c3(Chi, alpha):
    psi = alpha * Chi**2
    if abs(psi) < 1e-8: # Use series near 0 for numerical stability
        c2 = 0.5 - psi/24.0 + psi**2/720.0 - psi**3/40320.0
        c3 = 1.0/6.0 - psi/120.0 + psi**2/5040.0 - psi**3/362880.0
    elif psi > 0:
        s = np.sqrt(psi)
        c2 = (1.0 - np.cos(s)) / psi
        c3 = (s - np.sin(s)) / (s**3)
    else:
        s = np.sqrt(-psi)
        c2 = (np.cosh(s) - 1.0) / (-psi)
        c3 = (np.sinh(s) - s) / (s**3)
        
    return psi, c2, c3

def kepler_values(r0_vec, v0_vec, dt):
    '''
    Kepler propagation function as defined by Mathworks:
    https://www.mathworks.com/help/aerotbx/ug/orbit-pop-algorithms.html
    '''
    
    mu_earth = 3.986e14
    sqrt_mu = np.sqrt(mu_earth)
    
    r0 = np.linalg.norm(r0_vec)
    v0 = np.linalg.norm(v0_vec)
    vr0 = np.dot(r0_vec, v0_vec) / r0  # Radial velocity component v_r0 = (r0·v0)/|r0|
    
    epsilon = v0**2/2-mu_earth/r0 # determine orbital energy
    alpha = -2*epsilon/mu_earth # alpha should always be > 0 for an elliptical orbit
    Chi = sqrt_mu*dt*alpha # initial guess for first iteration
    psi, c2, c3 = psi_c2_c3(Chi, alpha)
    
    iterations = 0
    max_iter = 100000
    while(True):
        Chi_new = Chi + (sqrt_mu * dt - Chi**3*c3 - r0*vr0*Chi**2*c2/sqrt_mu-r0*Chi*(1-psi*c3)) / (Chi**2*c2 + r0*vr0*Chi*(1-psi*c3)/sqrt_mu + r0*(1-psi*c2))
        psi, c2, c3 = psi_c2_c3(Chi_new, alpha)
        
        if abs(Chi_new-Chi) < 1e-8:
            Chi = Chi_new
            break
        Chi = Chi_new # update Chi and iterate again
        
        iterations += 1
        if iterations > max_iter:
            print("Maximum iterations reached in Kepler Propagation. Exiting.")
            return None
        
    # calculate universal variables
    f = 1-Chi**2/r0*c2
    g = dt-Chi**3/sqrt_mu*c3
    
    r = f * np.asarray(r0_vec) + g * np.asarray(v0_vec)
    
    return r

def R3(theta): # used for Earth rotation calculations. Not a perfect model for Earth's rotation, but fast and good enough for overpass window determination.
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])

def time_to_overpass(fsw_obj, time_range_hours, max_distance, r_ECEF, v_ECEF, target_ECEF):
    '''
    A function to determine how long the spacecraft can enter low-power mode
    before it will be within range of a set of GPS coordinates for fine-pointing
    of antenna and transmission
    
    Parameters
    ----------
    time_range : time in hours up to which overpass possibility should be checked [hours]
    max_distance : maximum distance from target in meters [m]
    r_satellite: position relative to Earth in ECI
    v_satellite_ velocity relative to Earth in ECI   
    '''
    omega_earth = 7.2921150e-5
    large_dt = 60*5 # for coarse overpass determination (five minutes)
    small_dt = 5 # for fine overpass determination
    maximum_window = 3600 # large value to ensure exit boundary of overpass window is found
    exit_check_increment = 5 # search for window exit in increments
    lead_time = 100 # lead time before entering window to engage control system. Allows satellite to reorient in time.
    min_window_time = 305 # minimum overpass time
    skip_after_window = 2000 # time to skip after window with insufficient overpass time before searching again
    
    time = fsw_obj.skyfield_timescale.utc(fsw_obj.time_zero) 
    
    # itrs.rotation_at(t) returns the rotation matrix that maps ICRF/ECI -> ITRS/ECEF.
    R_ecef_from_eci = itrs.rotation_at(time)
    R_eci_from_ecef = R_ecef_from_eci.T # get current Earth rotation angle relative to ECI
    theta0 = np.arctan2(R_eci_from_ecef[1, 0], R_eci_from_ecef[0, 0]) # Get Earth's rotation angle in ECI at current time. Radians in [-pi, pi]. Uses NumPy indexing.
    
    # convert orbital vector parameters from ECEF to ECI
    r_ECI = R_eci_from_ecef @ r_ECEF
    v_ECI = R_eci_from_ecef @ v_ECEF
    
    def distance_to_target_eci(dt):
        r_eci = kepler_values(r_ECI, v_ECI, dt)
        theta = theta0 + dt * omega_earth
        target_eci = R3(theta) @ target_ECEF
        return np.linalg.norm(r_eci - target_eci)
    
    time_offset = 0 # window search offset
    t_end = time_range_hours*3600 # convert hours to seconds
    found_any_window = False # used to track second error type where overpass windows are found, but are not long enough
    while time_offset < t_end: # scan up until maximum time for a window which satisifies minimum time criteria
        
        # first coarse scan to find window
        overpass_time = None
        for dt in range(time_offset, t_end, large_dt): # check all future positions in propagation intervals
            if distance_to_target_eci(dt) <= max_distance:
                overpass_time = dt # we are passing over within range at this time
                found_any_window = True # used to log 
                break
        if overpass_time is None: # no overpass found within total time window, exit while loop
            break
        
        # second scan with finer interval to narrow down window-entry time
        lower_bound = max(overpass_time - large_dt, time_offset) # wind clock back by one time interval in order to scan in finer intervals
        upper_bound = overpass_time + 1 # include the final time step with +1
        
        window_start = None
        for dt in range(lower_bound, upper_bound, small_dt): # check all future positions in finer propagation intervals
            if distance_to_target_eci(dt) <= max_distance: 
                window_start = dt # save time at which SC is in range
                print(f"\nOverpass window entry predicted to occur in {window_start} seconds")
                break
        if window_start is None: # Guard against fine search not finding an overpass window (shouldn't be possible, but safety critical error)
            break
        
        # third step: find window exit time
        window_exit = None
        for dt in range(window_start, window_start+maximum_window, exit_check_increment): # check all future positions in propagation intervals
            if distance_to_target_eci(dt) > max_distance:
                window_exit = dt # window exit at this time
                print(f"Overpass window exit predicted to occur in {window_exit} seconds")
                break
        if window_exit is None: # Guard against exit not being found (could happen if algorithm is poorly tuned)
            window_exit = min(window_start + maximum_window, t_end)

        if (window_exit - window_start) >= min_window_time: # if an acceptable window has been found, return value
            controller_start = max(0, window_start - lead_time) # ensure controller can't see negative activation time (would work anyways in current implementatino, but this avoids future bugs)
            controller_end = window_exit
            return controller_start, controller_end
        else:
            print("Insufficient overpass duration")
        
        # Too short: advance offset and try again
        time_offset = window_exit + skip_after_window
    
    if not found_any_window:
        print(f"\n\nERROR: Overpass window not found in specified time window of {time_range_hours} hours!")
        print(f"Check if inclination allows for overpass within {max_distance/1e3} kilometers\n")
        return -1, None
    
    print(f"\n\nERROR: Sufficiently large overpass window not found in specified time window of {time_range_hours} hours!")
    print(f"Check if inclination allows for overpass within {max_distance/1e3} kilometers\n")
    return -2, None

def find_nearest_ground_station(fsw_obj, time_range_hours, max_distance, r_ECEF, v_ECEF):
    station_found = False # keep track of whether or not we find any of the listed stations in range
    chosen_station = None # which station we want to use
    next_overpass = None # how long until closest found overpass [seconds]
    for station in station_list:
        print(f"\nScanning {station.name}")
        start, end = time_to_overpass(fsw_obj, time_range_hours, max_distance, r_ECEF, v_ECEF, station.ECEF)
        if start == -1 or start == -2:
            start == None # deal with error messages
        if (start is not None) and ((next_overpass is None) or (start<next_overpass[0])):
            station_found = station
            chosen_station = station
            next_overpass = [start, end]
    
    if station_found:
        print(f"\nNext overpass opportunity is station {chosen_station.name}")
        print(f"Overpass window entry predicted to occur in {next_overpass[0]} seconds")
        print(f"Overpass window exit predicted to occur in {next_overpass[1]} seconds\n")
        return next_overpass[0], next_overpass[1]
    else:
        print("No overpasses found within range")
        return None, None
        
