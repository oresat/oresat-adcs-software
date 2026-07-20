from Basilisk.utilities import macros
import numpy as np
import basilisk_core
import json
from pathlib import Path

from config import GuidanceMode, PointingReference, ControlMode


def npdict_to_plaindict(npdict):
    """Convert numpy arrays of dictionary to list"""
    return {
        key: val.tolist() if isinstance(val, np.ndarray) else val 
        for key,val in npdict.items()
    }

def plaindict_to_npdict(plaindict):
    """Convert lists in dictionary to numpy arrays"""
    return {
        key: np.array(var) if isinstance(val, list) else val 
        for key,val in plaindict.items()
    }

if __name__ == "__main__":
    config_base_name = "Sentinel"
    # select satellite model attributes
    satellite = "Sentinel"
    # satellite = "SENTINEL"
    
    # select 3D model file
    if satellite == "Sentinel":
        sat_3D_file = "models/3U_Simplified_Model.obj"
        viz_scaling = 5 # 3D-model scaling factor
    
        # Inertia tensor data
        Jxx = 0.057
        Jxy = 0.01
        Jxz = 0.0
        Jyx = Jxy
        Jyy = 0.048
        Jyz = 0.0
        Jzx = Jxz
        Jzy = Jyz
        Jzz = 0.02
        
        R = R = np.diag([1, -1, -1])   # 180 deg rotation about x
        J = np.array([[Jxx, Jxy, Jxz], # satellite inertia matrix
                      [Jyx, Jyy, Jyz], 
                      [Jzx, Jzy, Jzz]])
        J = R @ J @ R.T
    else:
        sat_3D_file = "models/OreSat_Simplified_Model.obj"
        viz_scaling = 7 # 3D-model scaling factor
        
        # Inertia tensor data
        Jxx = 0.01650237
        Jxy = 0.00000711
        Jxz = 0.00004547
        Jyx = Jxy
        Jyy = 0.015962
        Jyz = 0.00003107
        Jzx = Jxz
        Jzy = Jyz
        Jzz = 0.00651814
    
        J = np.array([[Jxx, Jxy, Jxz], # satellite inertia matrix
                      [Jyx, Jyy, Jyz], 
                      [Jzx, Jzy, Jzz]])
    
    mass = 3.05353136 # satellite mass [kg]


    # SENSOR NOISE PARAMETERS
    # gyroscope
    sigma_gyro = 0.014 * macros.D2R # instantaneous white noise (datasheet gives value in degrees, convert to radians) (not sure which to use)
    sigma_bias = 1e-5 # slow random bias drift (random walk)
    P_b0 = 1 * macros.D2R # [rad/s] initial gyro uncertainty
    # star tracker
    sigma_ST = 2.4e-6 # [rad] measurement noise (instantaneous orientation error)
    P_ST_0 = 8.7e-7 # [rad^2] initial star tracker attitude uncertainty
    ST_update_rate = 1.1 # defined in seconds
    
 
    # Reaction Wheel Presets, add to satellite?
    # Define 4 reaction wheel unit vectors in a pyramid configuration (60 deg tilt from z-axis) 
    # rw_inertia = 4.2946e-6      # [kg*m^2], moment of inertia about spin axis (old values from OreSat 0.5 wheels)
    rw_inertia = 7.271e-6 # [kg*m^2] rotational inertia

    rw_z = np.cos(60*np.pi/180) # wheel angle from z axis. Same for all wheels
    rw_xy = np.cos(52.238756*np.pi/180) # wheel angle from x/y axis, sign varies by quadrant

    #  +x+y  +x-y  -x-y  -x+y  (motor positions in satellite quadrants. Each column represents one motor's torque components)
    # Wheel moment/orientation matrix
    rw_G = np.array(([[rw_xy,   rw_xy,  -rw_xy,  -rw_xy],
                   [rw_xy,  -rw_xy,  -rw_xy,   rw_xy],
                   [-rw_z,   -rw_z,   -rw_z,  -rw_z]])) 
    
    rw_max_speed = 11000.0 # ridiculous speed so our controller does the work. 100k effectively removes limit and allows fsw to limit manually.
    rw_max_torque = 100000.0 # only used when useMaxTorque = True. 100k effectively removes limit and allows fsw to limit manually.

    battery_capacity = 300000.0  # W*s according to basilisk
    battery_init_charge = 150000.0
    
    sat_config = {
        "satellite":satellite,  # satellite name
        "mass":mass, # satellite mass
        "J":J,  # rotational inertia matrix
        "sigma_gyro":sigma_gyro,  # gyroscope noise amplitude
        "sigma_bias":sigma_bias,  # bias wandering
        "P_b0":P_b0,  # initial gyro uncertainty
        "sigma_ST":sigma_ST,  # star tracker noise amplitude
        "P_ST_0":P_ST_0,  # initial star tracker uncertainty
        "ST_update_rate":ST_update_rate,  # (flightsoftware) how fast star tracker updates
        "battery_capacity": battery_capacity,  # battery capacity
        "battery_init_charge": battery_init_charge,  # initial charge of battery at start of mission
    }


    rw_config = {
        "rw_inertia": rw_inertia,
        "rw_G": rw_G,
        "rw_max_speed": rw_max_torque,
        "rw_max_torque": rw_max_torque,
    }

    blah = npdict_to_plaindict(sat_config)
    with open("example_satellite.json", "w") as fd:
        json.dump(blah, fd, indent=2)

    blah = npdict_to_plaindict(rw_config)
    with open("example_reaction_wheels.json", "w") as fd:
        json.dump(blah, fd, indent=2)


    # basilisk_core.sim_main(config)
