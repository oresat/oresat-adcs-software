import numpy as np
import time
from pathlib import Path
from sys import exit

import basilisk_core

import preset_utils

from config import GuidanceMode, PointingReference, ControlMode

if __name__ == "__main__":

    # select satellite model attributes
    sat_config = preset_utils.load_preset("presets/Osiris-C.json")

    # solar panels
    solar_config = preset_utils.load_preset("presets/solar_panels.json")

    # determine if hardware should have their own configs
    rw_config = preset_utils.load_preset("presets/reaction_wheels.json")

    # Select 3d file
    sat_3D_file = "models/3U_Simplified_Model.obj"

    # Simulation settings
    viz_scaling = 5

    viz_filename = None # sim visualization savename
    print_states = False # print states in flight software
    save_pdf = True # save plots as PDF's to target folder
    save_png = False # save plots as PNG's to target folder
    
    plot_basepath = Path("./figures") # path to which graphs should be saved
    use_filter = True # whether to use perfect state information or simulate with sensor noise and state estimation (MEKF)
    error_time_check = 100 # time after which maximum error is considered for evaluation

   
    # SIMULATION INITIAL STATE
    # initial satellite states
    init_rot_axis = [1, 0, 0] # this vector cannot be all zeros or quat.axis_angle_to_quaternion will return nan! 
    init_rot_angle = 0
    
    #omega_init_rpm = -np.array([1.5, 0.4, 0.7])  # initial spin rates [RPM]
    omega_init_rpm = -np.array([0.3, 0.2, 0.1])  # initial spin rates [RPM]
    omega_init_rad = omega_init_rpm * 2*np.pi/60  # convert RPM to rad/s

    # command rotations relative to initial orientation
    sat_rot_axis = [0, 1, 0]
    sat_rot_angle = 90
    

    # CONTROLLER
    # LQR tuning with or without integrator terms for steady state error corrections
    use_variable_gain = False 
    # only relevant for target pointing
    activate_on_overpass = False
    use_skyfield = True


    # MISSION PARMETERS
    time_init_string = "2026-02-10T20:00:00Z"

    # Select the spacecraft pointing reference (which axis/sensor defines boresight) and control modes:    
    pointing_reference = PointingReference["HELICAL"]
    
    control_mode = ControlMode["RW_POINTING"]

    # Track specified target on Earth's surface or nadir vector. Both with +x axis ram-facing.
    guidance_mode = GuidanceMode["NADIR"] # "NADIR" or "SUN" 

    # KSAT coordinates
    target_lat = 78.231500
    target_lon = 15.411100
    target_height = 488 # [m]
    
    # ESI headquarters coordinates
    # target_lat = 39.608251
    # target_lon = -104.895788
    # target_height = 1716 # [m]
    
     
    if control_mode in (ControlMode.RW_POINTING, ControlMode.THERMAL_DETUMBLE, ControlMode.RW_SLOW_ROTATE): # realistic RW sim setup
        sim_time = 5000
        dynamics_update_time = .2
        fsw_update_time = 1.0
        if (fsw_update_time > 2): # give user warning about unrealistic time steps so THEY DON'T WASTE TIME
            print("\nWARNING: FSW update time too large for stable convergence with reaction wheels\nExiting sim")
            exit()
    elif control_mode == ControlMode.IDLE:
        sim_time = 24000
        dynamics_update_time = 10
        fsw_update_time = 10
        use_filter = False
        use_skyfield = False
        omega_init_rad = np.array([0.0, 0.0, 0.0]) # ensure no excessive spinning
    else: # realistic MTB sim setup
        sim_time = 30000
        
        dynamics_update_time = 1
        fsw_update_time = 1
        
        #if control_mode == ControlMode.DETUMBLE:
        #    fsw_update_time = 2 # suggested fsw rate of no less than 5 seconds for stability when using MTB_POINTING, and no more than 2 when using DETUMBLE
        #else:
        #    fsw_update_time = 10
        
        activate_on_overpass = False
        
    if fsw_update_time < dynamics_update_time:
        fsw_update_time = dynamics_update_time # ensure flight software doesn't update more frequently than dynamics simulation
    if sat_config['ST_update_rate'] < fsw_update_time:
        sat_config['ST_update_rate'] = fsw_update_time # if ST update rate is faster than FSW, FSW throws an error
    
    

    sim_config = {
              "init_rot_axis":init_rot_axis, 
              "init_rot_angle":init_rot_angle, 
              "omega_init_rpm":omega_init_rpm, 
              "omega_init_rad":omega_init_rad,
              "sat_rot_axis":sat_rot_axis, 
              "sat_rot_angle":sat_rot_angle, 
              "pointing_reference":pointing_reference, 
              "control_mode":control_mode,
              "sim_time":sim_time, 
              "dynamics_update_time":dynamics_update_time, 
              "fsw_update_time":fsw_update_time, 
              "viz_filename":viz_filename, 
              "print_states":print_states,
              "save_pdf":save_pdf, 
              "save_png":save_png, 
              "plot_basepath":plot_basepath, 
              "use_filter":use_filter, 
              "error_time_check":error_time_check, 
              "guidance_mode":guidance_mode, 
              "target_lat":target_lat, 
              "target_lon":target_lon, 
              "target_height":target_height, 
              "sat_3D_file":sat_3D_file, 
              "viz_scaling":viz_scaling, 
              "use_skyfield":use_skyfield,
              "activate_on_overpass":activate_on_overpass, 
              "use_variable_gain":use_variable_gain,
              "time_init_string": time_init_string,
    }

    whole_config = sat_config | sim_config | solar_config | rw_config
 
    print(f"Satellite: {sat_config['satellite']}")
    print(f"Control Mode: {control_mode.name}")
    print(f"Pointing Reference: {pointing_reference.name}")
    print(f"Guidance Mode: {guidance_mode.name}")
    print(f"Activate On Overpass Mode: {activate_on_overpass}\n")

    delay_time_seconds = 3
    print(f"\nStarting simulation in {delay_time_seconds} seconds...")
    time.sleep(delay_time_seconds)

    basilisk_core.sim_main(whole_config)
