import numpy as np
import Quaternions as quat
from Basilisk.utilities import macros
from skyfield.framelib import itrs
from datetime import datetime, timezone

def GPS_to_ECEF(lat, lon, height, a, e2):
    sin_lat = np.sin(lat * macros.D2R)
    cos_lat = np.cos(lat * macros.D2R)
    
    N = a/np.sqrt(1-e2*sin_lat**2) # lattitude must be signed for WGS-84
    x = (N+height)*cos_lat*np.cos(lon * macros.D2R)
    y = (N+height)*cos_lat*np.sin(lon * macros.D2R)
    z = (N*(1-e2)+height)*sin_lat

    return np.asarray([x, y, z])

def target_tracking_quat(fsw_obj, target_vector, v_ECEF, ECI_2_ECEF):
    '''
    Creates an orientation quaternion forming an orientation based on a target
    vector for the z-facing, and orients the +x facing as close as possible to
    the velocity vector
    
    Used for target tracking and nadir pointing
    '''
    
    R_NE = ECI_2_ECEF.T # rotation matrix from ECEF to ECI
    v_ECI = R_NE @ (v_ECEF/np.linalg.norm(v_ECEF)) # norm velocity vector and convert to ECI
        
    zvec = R_NE @ (target_vector/np.linalg.norm(target_vector)) # norm target vector and convert to ECI
    
    xvec = v_ECI - np.dot(v_ECI, zvec) * zvec # remove component parallel to nadir vector from velocity vector to determine "ram-facing-like" vector
    xvec = xvec/np.linalg.norm(xvec) # norm

    yvec = np.cross(zvec, xvec)
    yvec = yvec/np.linalg.norm(yvec) # norm
    
    # xvec = np.cross(yvec, zvec) # Re-orthogonalize zB to avoid numerical drift
    # xvec = xvec/np.linalg.norm(xvec) # norm
    
    C_BN = np.vstack((xvec, yvec, zvec)) # Create DCM for body orientation in ECI coordinates
    
    target_quat = quat.quat_from_dcm_scalar_last(C_BN) # Convert DCM to quaternion
    fsw_obj.update_target(target_quat) # update FSW target
    
def ram_quaternion(fsw_obj, drag_orientation, v_ECEF, nadir_vec, ECI_2_ECEF):
    '''
    Creates an orientation quaternion forming an orientation based on
    whether maximum or minimum drag is desired. The secondary axis is defined
    as the nadir vector, or as close as possible to it
    '''
    
    R_NE = ECI_2_ECEF.T
    drag_facing = R_NE @ (v_ECEF/np.linalg.norm(v_ECEF)) # norm velocity vector and convert to ECI
    
    nadir_ECI = R_NE @ nadir_vec
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
    fsw_obj.update_target(target_quat) # update FSW target
    
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
    
    while(True):
        Chi_new = Chi + (sqrt_mu * dt - Chi**3*c3 - r0*vr0*Chi**2*c2/sqrt_mu-r0*Chi*(1-psi*c3)) / (Chi**2*c2 + r0*vr0*Chi*(1-psi*c3)/sqrt_mu + r0*(1-psi*c2))
        psi, c2, c3 = psi_c2_c3(Chi_new, alpha)
        
        if abs(Chi_new-Chi) < 1e-8:
            Chi = Chi_new
            break
        Chi = Chi_new # update Chi and iterate again
        
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
    
def time_to_overpass(fsw_obj, currentTimeNanos, time_range, max_distance, r_satellite, v_satellite, target):
    '''
    A function to determine how long the spacecraft can enter low-power mode
    before it will be within range of a set of GPS coordinates for fine-pointing
    of antenna and transmission
    
    Parameters
    ----------
    time_range : time in hours up to which overpass possibility should be checked [hours]
    max_distance : maximum distance from target in meters [m]
    
    '''
    
    large_time_delta = 60*5 # for coarse overpass determination (five minutes)
    small_time_delta = 5 # for fine overpass determination
    
    time = fsw_obj.skyfield_timescale.utc(fsw_obj.time_zero) 
    
    # itrs.rotation_at(t) returns the rotation matrix that maps ICRF/ECI -> ITRS/ECEF.
    R_ecef_from_eci = itrs.rotation_at(time)
    R_eci_from_ecef = R_ecef_from_eci.T # get current Earth rotation angle relative to ECI
    theta0 = np.arctan2(R_eci_from_ecef[1, 0], R_eci_from_ecef[0, 0]) # Get Earth's rotation angle in ECI at current time. Radians in [-pi, pi].
    
    for dt in range(0, time_range*3600, large_time_delta): # check all future positions in propagation intervals
        r_new = kepler_values(r_satellite, v_satellite, dt) # get Kepler values
        
        theta_new = theta0 + dt*fsw_obj.omega_earth # calculate Earth's rotation after dt
        ECI_target = R3(theta_new) @ target # rotate target position into ECI based on new theta
        if (np.linalg.norm(r_new-ECI_target) <= max_distance):
            overpass_time = dt # we are passing over within range at this time
            break
        
    lower_bound = overpass_time - large_time_delta # wind clock back by one time interval in order to scan in finer intervals
    upper_bound = lower_bound + large_time_delta # same as found overpass time. Coded for easier reading.
            
    for dt in range(lower_bound, upper_bound, small_time_delta): # check all future positions in finer propagation intervals
        r_SC_new = kepler_values(r_satellite, v_satellite, dt) # get new spacecraft position
        
        theta_new = theta0 + dt*fsw_obj.omega_earth # calculate Earth's rotation after dt
        ECI_target = R3(theta_new) @ target # rotate target position into ECI based on new theta
        if (np.linalg.norm(r_SC_new-ECI_target) <= max_distance): 
            window_start_delay = dt # save time at which SC is in range
            break
        
    fsw_obj.controllerStartTime = window_start_delay-100 # subtract 100 seconds to allow spacecraft to reorient and stabilize in time for overpass window
    print(f"\nOverpass window entry predicted to occur in {window_start_delay} seconds")
    
    maximum_window = 3600 # large value to ensure exit boundary of overpass window is found
    exit_check_increment = 20 # search for window exit in increments
    
    for dt in range(window_start_delay, maximum_window, exit_check_increment): # check all future positions in propagation intervals
        r_SC_new = kepler_values(r_satellite, v_satellite, dt) # get new spacecraft position
        
        theta_new = theta0 + dt*fsw_obj.omega_earth # calculate Earth's rotation after dt
        ECI_target = R3(theta_new) @ target # rotate target position into ECI based on new theta
        if (np.linalg.norm(r_SC_new-ECI_target) > max_distance):
            window_exit_delay = dt # window exit at this time
            break
    
    fsw_obj.controllerEndTime = window_exit_delay
    print(f"Overpass window exit predicted to occur in {window_exit_delay} seconds\n")

    # SIMULATE WHEEL SHUTDOWN