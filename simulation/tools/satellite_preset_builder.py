from Basilisk.utilities import macros
import numpy as np
import json
from pathlib import Path


import preset_utils


if __name__ == "__main__":
    config_base_name = "Prism"
    # select satellite model attributes
    satellite = "Prism"
    
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
    elif satellite == "Prism":
        sat_3D_file = "models/3U_Simplified_Model.obj"
        viz_scaling = 5 # 3D-model scaling factor
    
        # Inertia tensor data
        Jxx = 0.031
        Jxy = 0.000001
        Jxz = 0.000001
        Jyx = Jxy
        Jyy = 0.025
        Jyz = 0.000001
        Jzx = Jxz
        Jzy = Jyz
        Jzz = 0.012

        # 180 deg rotation about x
        # wait why is there a rotation?
        R = R = np.diag([1, -1, -1])
        J = np.array([[Jxx, Jxy, Jxz], # satellite inertia matrix
                      [Jyx, Jyy, Jyz], 
                      [Jzx, Jzy, Jzz]])
        J = R @ J @ R.T

    elif satellite == "Beecon":
        sat_3D_file = "models/3U_Simplified_Model.obj"
        viz_scaling = 5 # 3D-model scaling factor
    
        # Inertia tensor data
        Jxx = 0.031
        Jxy = 0.000001
        Jxz = 0.000001
        Jyx = Jxy
        Jyy = 0.025
        Jyz = 0.000001
        Jzx = Jxz
        Jzy = Jyz
        Jzz = 0.012

        # 180 deg rotation about x
        # wait why is there a rotation?
        R = R = np.diag([1, -1, -1])
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

    rw_base_power_rate = 0.2  # base power consumption in watts
    rw_eff_elec_to_mech = 0.8  # Efficiency 
    rw_eff_mech_to_elec = -1.0  # Recovery, negative means it takes energy to brake

    # expects single 1x(3*n) array
    mt_qty = 3
    mt_G = np.array([
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    ])
    mt_max_dipoles = [2.326784361405822, 2.326784361405822, 0.37338038792999995]


    battery_mah = 2600
    battery_voltage = 3.6
    # each battery pack has two (18650) batteries
    # each card as two packs (four 18650 batteries)
    # a satellite may have two battery cards (8 18650 batteries)
    num_batteries = 8

    # Joules (basilisk example started with 300,000 joules
    # the calculation is about 269,568 joules
    battery_capacity = battery_mah * battery_voltage * 3.6 * num_batteries
    # Joules
    battery_init_charge = 0.5 * battery_capacity

    battery_capacity = 300000.0  # Joules according to basilisk
    battery_init_charge = 150000.0  # Joules according to basilisk
    # for batteries, do mAh * V * 3.6



    base_power_rate = -5.0  # Watts, negative means it consumes power

    
    solar_config = {
        "solar_panel_params": [
            {
                "sp_norm": [1, 0, 0], 
                "sp_area": 0.03096768, 
                "sp_eff": 0.20
            },
            {
                "sp_norm": [0, 1, 0], 
                "sp_area": 0.03096768, 
                "sp_eff": 0.20
            },
            {
                "sp_norm": [-1, 0, 0], 
                "sp_area": 0.03096768, 
                "sp_eff": 0.20
            }
        ]
    }

    print(solar_config["solar_panel_params"])
    blah = [
        [val for val in panel.values()] 
        for panel in solar_config["solar_panel_params"]
    ]

    print(blah)

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
        "base_power_rate": base_power_rate,  # rate at which power is consumed
    }

    rw_config = {
        "rw_inertia": rw_inertia,
        "rw_G": rw_G,
        "rw_max_speed": rw_max_torque,
        "rw_max_torque": rw_max_torque,
        "rw_base_power_rate": rw_base_power_rate,
        "rw_eff_elec_to_mech": rw_eff_elec_to_mech,
        "rw_eff_mech_to_elec": rw_eff_mech_to_elec,
    }
    mt_config = {
        "mt_qty": mt_qty,
        "mt_G": mt_G,
        "mt_max_dipoles": mt_max_dipoles
    }

    blah = preset_utils.npdict_to_plaindict(sat_config)
    with open("example_satellite.json", "w") as fd:
        json.dump(blah, fd, indent=2)

    blah = preset_utils.npdict_to_plaindict(rw_config)
    with open("example_reaction_wheels.json", "w") as fd:
        json.dump(blah, fd, indent=2)

    blah = preset_utils.npdict_to_plaindict(mt_config)
    with open("example_magnetorquers.json", "w") as fd:
        json.dump(blah, fd, indent=2)


    blah = preset_utils.npdict_to_plaindict(solar_config)
    with open("example_solar_panels.json", "w") as fd:
        json.dump(blah, fd, indent=2)


