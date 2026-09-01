import numpy as np
import time
from Basilisk.simulation import spacecraft, starTracker, imuSensor, reactionWheelStateEffector, magneticFieldWMM, magnetometer, MtbEffector # import simulation related support
from Basilisk.utilities import SimulationBaseClass, macros, vizSupport, simIncludeGravBody, orbitalMotion, simIncludeRW, simHelpers # import general simulation support files
from Basilisk.architecture import messaging
from Basilisk import __path__
from Basilisk.utilities import RigidBodyKinematics as rbk
from Plotting_Functions import plot_rw_speeds, plot_magfield, plot_imu
from FlightSoftwareModule import FlightSoftware # self defined module to emulate flight software ADCS tasks
import quaternion as quat
from pathlib import Path
from sys import exit
bskPath = __path__[0]

import matplotlib.pyplot as plt
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile


# similar file of what is in oresat-simulator
import basilisk_helpers as bsk_helpers


from config import GuidanceMode, PointingReference, ControlMode


# EXAMPLE FOR READING BASILISK MESSAGING SYSTEM AND ATTRIBUTES. USED FOR DOCUMENTATION, DO NOT DELETE!!!
# message = scObject.scStateOutMsg.read()   # works before or after ExecuteSimulation()
# print(dir(message))

def sim_main(config):
    # simulation variables
    dynamics_update_time = config["dynamics_update_time"] # seconds
    fsw_update_time = config["fsw_update_time"] # temporarily REALLY small to make the system respond as intended
    
    # Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(True) # creates simulation progress bar output

    # create the simulation processes
    dynProcess = sim.CreateNewProcess("dynamicsProcess") # physical simulation of satellite
    fswProcess = sim.CreateNewProcess("fswProcess") # flight software simulation
    
    # create the dynamics & flight software tasks and specify their respective integration update times
    dynProcess.addTask(sim.CreateNewTask("dynamicsTask", macros.sec2nano(dynamics_update_time)))
    fswProcess.addTask(sim.CreateNewTask("fswTask", macros.sec2nano(fsw_update_time)))
    
    ############################## SPACECRAFT #################################
    
    # create spacecraft object
    scObject = spacecraft.Spacecraft() # initialize object
    scObject.ModelTag = config["satellite"] # name object
    scObject.hub.mHub = config["mass"]  # [kg], not sure this is required given inertial space, but should be realistic
    scObject.hub.IHubPntBc_B = config["J"] # assign OreSat inertia matrix
    init_rot_axis = np.array(config["init_rot_axis"])
    init_rot_angle = np.array(config["init_rot_angle"])
    initial_MRP = (init_rot_axis/np.linalg.norm(init_rot_axis)) * np.tan(init_rot_angle*macros.D2R/4.0) # MRP set to customize initial inertial attitude
    scObject.hub.sigma_BNInit = initial_MRP
    scObject.hub.omega_BN_BInit = config["omega_init_rad"]

    sc_object_msg = scObject.scStateOutMsg

    sc_object_rec = sc_object_msg.recorder()



    ########################## ORBITAL ENVIRONMENT ############################
    
    # create gravitational bodies (Earth in this case, but might add moon later as well)
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createBodies('earth')

    mu_earth = gravFactory.gravBodies.get("earth").mu


    # create orbit properties using classical orbit elements. 
    # Assuming perfectly circular orbit for now.
    oe = orbitalMotion.ClassicElements()
    oe.a = (415+6371) * 1e3 # semi-major axis  [meters] (altitude + earth's radius)
    oe.e = 0 # eccentricity
    oe.i = 50 * macros.D2R # inclination [rad]
    oe.Omega = 0 * macros.D2R  # RAAN or Longitude of the Ascending Node [rad]
    oe.omega = 0.0 * macros.D2R  # argument of periapsis [rad]
    oe.f = 10 * macros.D2R       # true anomaly [rad]
    
    # true orbit parameters for SENTINEL mission
    oe = orbitalMotion.ClassicElements()
    Re = 6371e3 # radius of Earth
    apoapsis = 600e3
    periapsis = 580e3
    ra = Re + apoapsis
    rp = Re + periapsis
    oe.a = 0.5*(rp + ra)
    oe.e = (ra - rp)/(ra + rp)
    oe.i = 98.7 * macros.D2R # [degrees]
    oe.Omega = 130 * macros.D2R # [degrees]
    oe.omega = 0 * macros.D2R   # sets perigee vector angle from ascending node in the orbital plane [degrees]
    oe.f = 82 * macros.D2R      # where the satellite is on the ellipse at epoch (start of sim) [degrees]
    
    rN, vN = orbitalMotion.elem2rv(mu_earth, oe)
    oe = orbitalMotion.rv2elem(mu_earth, rN, vN)  # this stores consistent initial orbit elements, fixes numerical errors, particulary with perfectly circular orbits. Consult ChatGPT for detailed explanation.
    orbital_period = 2*np.pi*np.sqrt(oe.a**3/mu_earth) # define orbital period for plotting
    orbital_inclination = oe.i # used for readable argument passing when defining FSW
    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # r_BN_N [m]
    scObject.hub.v_CN_NInit = vN  # v_BN_N [m/s]


    # Add spice object for planet rotation and ECEF coordinate simulation. Necessary for guidance algorithms.
    timeInitString = config["time_init_string"]
    
    grav_factory, spice_object, earth, sun, moon = bsk_helpers.make_spice_earth_sun_moon(scObject, timeInitString)

    grav_factory.addBodiesTo(scObject)

    earth_msg = spice_object.planetStateOutMsgs[0]
    sun_msg = spice_object.planetStateOutMsgs[1]
    moon_msg = spice_object.planetStateOutMsgs[2]
    
    earth_rec = spice_object.planetStateOutMsgs[0].recorder()
    sun_rec = spice_object.planetStateOutMsgs[1].recorder()
    moon_rec = spice_object.planetStateOutMsgs[2].recorder()

    # Task ordering: SPICE before modules that consume its outputs. 
    # Need spice to run before spacecraft module.
    # priority could also be changed
    sim.AddModelToTask("dynamicsTask", spice_object, 5)
    # another way to add is grav_factory.spiceObject


    sim.AddModelToTask("dynamicsTask", scObject)
    sim.AddModelToTask("dynamicsTask", sc_object_rec)

    # used to record ECEF coordinates
    sim.AddModelToTask("dynamicsTask", earth_rec)
    sim.AddModelToTask("dynamicsTask", sun_rec)
    sim.AddModelToTask("dynamicsTask", moon_rec)



    # Eclipse Model
    eclipse_model, eclipse_msg, eclipse_rec = bsk_helpers.get_eclipse_multiple_model(
        model_tag = "eclipse_model",
        sc_object_msg = sc_object_msg,
        sun_msg = sun_msg,
        planet_msgs = [earth_msg, moon_msg]
    )
    sim.AddModelToTask("dynamicsTask", eclipse_model)
    sim.AddModelToTask("dynamicsTask", eclipse_rec)

    # Magnetic field model
    epoch_msg = simHelpers.timeStringToGregorianUTCMsg('2025 June 27, 10:23:0.0 (UTC)')  # set epoch date/time message for WMM
    
    # create the magnetic field
    mag_model, mag_msg, mag_rec = bsk_helpers.get_mag_model("WMM", scObject)
    # connect epoch messages
    mag_model.epochInMsg.subscribeTo(epoch_msg) 
    # add the magnetic field module to the simulation task
    sim.AddModelToTask("dynamicsTask", mag_model) 
    sim.AddModelToTask("dynamicsTask", mag_rec)


    ############################### SENSORS ###################################
    
    # Create and configure a star tracker
    # Define dcm_CB for star tracker orientation (body-to-case, star tracker on +x side)
    dcm_CB = np.array([[0.0, 0.0, -1.0], # x_B -> -z_C
                       [0.0, 1.0, 0.0],  # y_B -> y_C
                       [1.0, 0.0, 0.0]]) # z_B -> x_C

    noise_std = config["sigma_ST"] if config["use_filter"] else None
    w_bounds = 1 if config["use_filter"] else None

    starTrackerSensor, starTrackerRec = bsk_helpers.make_star_tracker(modelTag="starTracker", 
                                                                      scObjMsg=scObject.scStateOutMsg,
                                                      record_t=macros.sec2nano(fsw_update_time),
                                                      dcm_CB=dcm_CB,
                                                      noise_std=noise_std,
                                                      w_bounds=w_bounds)
    
    sim.AddModelToTask("fswTask", starTrackerSensor) # Add sensor to flight software task
    sim.AddModelToTask("fswTask", starTrackerRec) # Add recording to task
   


    # Create and configure IMU sensor
    noise_std = config["sigma_gyro"] if config["use_filter"] else None
    e_bounds = 5 if config["use_filter"] else None

    imu, imuRec = bsk_helpers.make_gyro(modelTag="imu",
                                       scObjMsg=scObject.scStateOutMsg,
                                       record_t=macros.sec2nano(fsw_update_time),
                                       dcm=None,
                                       noise_std=noise_std,
                                       e_bounds=e_bounds,
                                       w_bounds=None)

    sim.AddModelToTask("fswTask", imu) # Add sensor to flight software task
    sim.AddModelToTask("fswTask", imuRec) # Add recording to task
   


    # Create magnetometer sensor
    mag_sensor, mag_sensor_rec = bsk_helpers.make_magnetometer(
        model_tag="TAM_sensor",
        mag_msg = mag_model.envOutMsgs[0], # is this reading only the first element?
        sc_object_msg = scObject.scStateOutMsg,
        record_t = macros.sec2nano(fsw_update_time),
        dcm = None,
        noise_std = config["tam_noise_std"],
        bias = config["tam_bias"],
        w_bounds = config["tam_w_bounds"]
    )

    sim.AddModelToTask("fswTask", mag_sensor)
    sim.AddModelToTask("fswTask", mag_sensor_rec) # Add recording to task
   

    ############################## EFFECTORS ##################################

    # Create reaction wheels
    
    RWFactory, rwStateEffector = bsk_helpers.make_rw_set(
        model_tag = "RW_cluster",
        sc_object = scObject,
        rw_G = config["rw_G"],
        rot_inertia = config["rw_inertia"],
        use_max_torque = True,
        max_speed = config["rw_max_speed"],
        max_torque = config["rw_max_torque"],
        has_jitter = False,
    )

    RWFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)
    sim.AddModelToTask("dynamicsTask", rwStateEffector)
   
    # log reaction wheel behavior
    rw_speed_rec = rwStateEffector.rwSpeedOutMsg.recorder()
    sim.AddModelToTask("dynamicsTask", rw_speed_rec)


    # counts the number of registered reaction wheels
    numRW = RWFactory.getNumOfDevices() 

    # create magnetic torque bar (MTB) object
    mtbEff = MtbEffector.MtbEffector()
    mtbEff.ModelTag = "MtbEff"
    scObject.addDynamicEffector(mtbEff)
    sim.AddModelToTask("dynamicsTask", mtbEff)
    mtbLog = mtbEff.mtbOutMsg.recorder()
    sim.AddModelToTask("dynamicsTask", mtbLog)

    mtbConfigParams = messaging.MTBArrayConfigMsgPayload()
    mtbConfigParams.numMTB = config["mt_qty"]
    mtbConfigParams.GtMatrix_B = config["mt_G"]
    mtbConfigParams.maxMtbDipoles = config["mt_max_dipoles"]
    #[2.326784361405822, 2.326784361405822, 0.37338038792999995] # individual rod Dipole limits when using current of 0.1 Amps [A·m^2]

    mtbCfgMsg = messaging.MTBArrayConfigMsg().write(mtbConfigParams)
    
    mtbCmd = messaging.MTBCmdMsgPayload()
    mtbCmd.mtbDipoleCmds = [0.0] * mtbConfigParams.numMTB
    mtbCmdMsg = messaging.MTBCmdMsg().write(mtbCmd)
    
    # subscribe messages
    mtbEff.mtbParamsInMsg.subscribeTo(mtbCfgMsg)
    mtbEff.mtbCmdInMsg.subscribeTo(mtbCmdMsg)
    mtbEff.magInMsg.subscribeTo(mag_model.envOutMsgs[0])  # from WMM module


    ############################ ENERGY ANLYSIS ###############################

    # Create a power montior aka battery
    battery, battery_msg, battery_rec = bsk_helpers.make_power_monitor(
        model_tag = "battery_1",
        storage_capacity = config["battery_capacity"],
        init_charge = config["battery_init_charge"],
    )
    sim.AddModelToTask("dynamicsTask", battery)
    sim.AddModelToTask("dynamicsTask", battery_rec)

    # create base power sink
    power_sink, power_sink_msg, power_sink_rec = bsk_helpers.make_power_sink(
        model_tag = "base_power_sink",
        node_power_out = config["base_power_rate"]
    )
    sim.AddModelToTask('dynamicsTask', power_sink)
    sim.AddModelToTask('dynamicsTask', power_sink_rec)
    battery.addPowerNodeToModel(power_sink_msg)

    # Solar Panels
    solar_panel_models = []
    solar_panel_msgs = []
    solar_panel_recs = []
    for ii, sp_params in enumerate(config["solar_panel_params"]):
        sp_model, sp_msg, sp_rec = bsk_helpers.make_solar_panel(
            model_tag = "solal_panel_" + str(ii),
            sc_object_msg = sc_object_msg,
            sun_msg = sun_msg,
            eclipse_msg = eclipse_msg,
            parameters = sp_params.values()
        )
        solar_panel_models.append(sp_model)
        solar_panel_msgs.append(sp_msg)
        solar_panel_recs.append(sp_rec)
        sim.AddModelToTask("dynamicsTask", sp_model)
        sim.AddModelToTask("dynamicsTask", sp_rec)
        battery.addPowerNodeToModel(sp_msg)


    # Create list of reaction wheel power models
    # assume all reaction wheels have the same characteristics
    rw_power, rw_power_msgs, rw_power_recs = bsk_helpers.make_rw_powers(
        model_tag = "rw_power",
        num_wheels = numRW,
        base_power = config["rw_base_power_rate"],
        eff_elec_to_mech = config["rw_eff_elec_to_mech"],  # should be about 0.8
        eff_mech_to_elec = config["rw_eff_mech_to_elec"],  # should be about -1.0 (energy to brake)
    )

    # attach reaction wheel powers to their things
    for ii in range(numRW):
        # listen to RW state effector
        rw_power[ii].rwStateInMsg.subscribeTo(rwStateEffector.rwOutMsgs[ii])
        # send output messages to power_monitor / battery
        battery.addPowerNodeToModel(rw_power_msgs[ii])
        # add power models to simulation
        sim.AddModelToTask("dynamicsTask", rw_power[ii])
        # add recorders to simulation
        sim.AddModelToTask("dynamicsTask", rw_power_recs[ii])



    ############################ FLIGHT SOFTWARE ##############################
    # update config dict
    config.update({"orbital_period":orbital_period, "orbital_inclination":orbital_inclination})


    # Create flight software object and subscribe all sensors
    fsw = FlightSoftware(config) # Create flight software object. Model tag already defined in __init__ as flight_software
    fsw.starTrackerMsgIn.subscribeTo(starTrackerSensor.sensorOutMsg) # subscribe to star tracker messages
    fsw.imuMsgIn.subscribeTo(imu.sensorOutMsg) # subscribe to IMU messages
    fsw.rwSpeedMsgIn.subscribeTo(rwStateEffector.rwSpeedOutMsg) # subscribe fsw reaction wheel speed input to reaction wheel output
    fsw.magMsgIn.subscribeTo(mag_sensor.tamDataOutMsg) # subscribe fsw to magenotometer readings
    fsw.scStateIn.subscribeTo(scObject.scStateOutMsg) # subscribe fsw to spacecraft state (positioning) for GPS emulation
    fsw.earthStateInMsg.subscribeTo(spice_object.planetStateOutMsgs[0]) # subscribe fsw to earth SPICE data, necessary to extract J20002PFix transformation matrix
    fsw.set_time_zero_from_iso_utc(timeInitString) # initialize ephemeris start time for GPS timestamp emulation
    sim.AddModelToTask("fswTask", fsw)
    
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(fsw.rwMotorTorqueOutMsg) # subscribe reaction wheel command input to flight software control output
    mtbEff.mtbCmdInMsg.subscribeTo(fsw.mag_dipole_msg)  # subscribe magnetorquer command input to flight software control output
    
    # determine initial pointing vector for relative target calculations
    sat_q_init = quat.axis_angle_to_quaternion(init_rot_axis, init_rot_angle) # account for any rotations of the satellite it self at sim initialization
    if fsw.pointing_reference == PointingReference.STAR_TRACKER:
        q_init = quat.quat_mult(quat.axis_angle_to_quaternion([0,1,0], 90), sat_q_init)
    elif fsw.pointing_reference == PointingReference.HELICAL:
        q_init = sat_q_init
    elif fsw.pointing_reference == PointingReference.CIRRUS_FLUX:
        q_init = quat.quat_mult(quat.axis_angle_to_quaternion([0,1,0], 180), sat_q_init)
    else:
        print("ERROR: Invalid pointing reference selected!")
        exit()

    q_rot = quat.axis_angle_to_quaternion(config["sat_rot_axis"], config["sat_rot_angle"])
    fsw.update_target(quat.quat_mult(q_rot, q_init))
    
    # print(f"\nSatellite view device is \"{fsw.pointing}\" with initial reference: {q_init}")
    # print(f"Satellite initial pointing target: {fsw.q_target}\n")




    ############################## SIMULATION #################################

    # add spacecraft state recording in order to read attitudes for plotting
    stateRec = scObject.scStateOutMsg.recorder(macros.sec2nano(dynamics_update_time)) # create dynamics recorder
    sim.AddModelToTask("dynamicsTask", stateRec) # add recorder to dynamics simulation
     
    if config["viz_filename"]:
        fileName = "./{viz_filename}"
    else:
        fileName = __file__
    
    current_dir = Path(__file__).parent.resolve() # find current working directory such that any system running code directly from git can use the simplified model
    model_file_path = current_dir / config["sat_3D_file"]



    viz = vizSupport.enableUnityVisualization(sim, "dynamicsTask", scObject, 
                                              saveFile=fileName, 
                                              liveStream=False, # let Vizard visualize data
                                              rwEffectorList=rwStateEffector) # add reaction wheel list to visualization


    # add pointing lines
    vizSupport.createPointLine(viz, toBodyName='earth', lineColor='green')
    vizSupport.createPointLine(viz, toBodyName='sun', lineColor='yellow')

    vizSupport.addLocation(viz,
        stationName="Boulder Station", 
        parentBodyName=earth.displayName,
        lla_GP = [
            np.radians(config["target_lat"]), 
            np.radians(config["target_lon"]), 
            config["target_height"]
        ],
        fieldOfView=np.radians(160.),
        color='pink',
        range=2000.0*1000  # meters
    )

    vizSupport.setActuatorGuiSetting(viz, viewRWPanel=True, viewRWHUD=True)
    s_factor = config["viz_scaling"] # 3D-model scaling factor
    vizSupport.createCustomModel(viz,
                                 modelPath=str(model_file_path), # Vizard expects filepath as a string
                                 scale=[-s_factor, s_factor, s_factor], # scale model and mirror on x-axis (don't know why the model is otherwise improperly mirrored)
                                 rotation=[0,np.pi/2,np.pi/2]) # rotate to properly align body axes with simulation axes
    
    # simulate:
    sim.InitializeSimulation() # initialize simulation
    sim.ConfigureStopTime(macros.sec2nano(config["sim_time"])) # configure a simulation stop time
    print("\nSimulation setup complete\nBeginning simulation\n")
    start = time.time()
    sim.ExecuteSimulation() # execute simulation
    end = time.time()
    

    ############################ POST PROCESSING ##############################
    '''
    Data arrays are defined differently in this section depending on whether
    the data originates from a dynamics process or a flightsoftware process, which
    typically run at different rates in the simulation. As such, their lengths
    must be adapted to be plottable by upsampling the shorter array.
    '''

    # save data
    # degrees
    rotation_raw_data = sc_object_rec.omega_BN_B

    # position data
    position_eci_data = sc_object_rec.r_BN_N
    # this is inertial reference frame, not ECEF
    # rotation will be based on time
    np.savetxt("output_data/position_eci_data.csv", position_eci_data, delimiter=",")

    # get earth rotation data
    eci_2_ecef_data = earth_rec.J20002Pfix

    position_ecef_data = np.zeros(position_eci_data.shape)
    for ii in range(position_eci_data.shape[0]):
        position_ecef_data[ii] = eci_2_ecef_data[ii] @ position_eci_data[ii]

    np.savetxt("output_data/position_ecef_data.csv", position_ecef_data, delimiter=",")

    rotation_deg_data = imuRec.AngVelPlatform * 180 / np.pi
    # Magnetic data is magnetic field, not in satellite reference frame
    magnetic_data = mag_rec.magField_N

    # Eclipse data is shadow factor
    eclipse_data = eclipse_rec.shadowFactor

    mag_sensor_data = mag_sensor_rec.tam_S

    np.savetxt("output_data/mag_field.csv", magnetic_data, delimiter=",")
    np.savetxt("output_data/mag_sensor.csv", mag_sensor_data, delimiter=",")

    # Magnetorquers torques
    mt_torque_data = mtbLog.mtbNetTorque_B
    # Reaction wheel speeds
    rw_speed_data = rw_speed_rec.wheelSpeeds

    # Battery capacity
    batt_storage_data = battery_rec.storageLevel
    batt_power_data = battery_rec.currentNetPower

    # Solar panel power
    sp_group_data = [sp_pow_rec.netPower for sp_pow_rec in solar_panel_recs]
    # Reaction wheel power
    rw_power_group_data = [rw_pow_rec.netPower for rw_pow_rec in rw_power_recs]
    # Should save data to file for postprocessing

    fig, ax = plt.subplots(figsize=(8,4))
    ax.plot(mtbLog.times() *1e-9, mt_torque_data)
    ax.set_ylim([-1e-6, 1e-6])
    plt.title("Magnetorquers: Net Torque")
    # plt.legend()

    # Plot reaction wheel torques
    if config["save_pdf"] == True:
        pdf_path = config["plot_basepath"] / "mt_torque_graph.pdf"
        plt.savefig(pdf_path, dpi=300)
        print(f"Plot saved as {pdf_path}")
    if config["save_png"] == True:
        png_path = config["plot_basepath"] / "mt_torque_graph.png"
        plt.savefig(png_path, dpi=600)
        print(f"Plot saved as {png_path}")

    # compare actual mag field to sensor for noise
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 3, 1)
    ax1.plot(np.linalg.norm(mag_sensor_data, axis=1))
    
    ax2 = fig.add_subplot(1, 3, 2)
    ax2.plot(np.linalg.norm(magnetic_data, axis=1))

    ax3 = fig.add_subplot(1, 3, 3)
    mag_sensor_magnitude = np.repeat(
        np.linalg.norm(mag_sensor_data, axis=1)[:-1], 
        int(fsw_update_time/dynamics_update_time), 
        axis=0
    )
    mag_field_magnitude = np.linalg.norm(magnetic_data, axis=1)[:-1]

    print(mag_sensor_magnitude.shape)
    print(mag_field_magnitude.shape)
    ax3.plot(mag_sensor_magnitude)
    ax3.plot(mag_field_magnitude, linewidth=5)


    # Magnetorquer detumble effectiveness
    if config["control_mode"] == ControlMode.DETUMBLE: 
        fig, ax = plt.subplots(3, 2, figsize=(8,6))
        #ax[0, 0].plot(mtbLog.times() *1e-9, np.linalg.norm(mt_torque_data, axis=1))
        #ax[0, 0].set_title("MT Torque Magnitude")
        ax[0, 0].plot(fsw.mt_cmd_history)
        ax[0, 0].plot(np.linalg.norm(fsw.mt_cmd_history, axis=1))
        ax[0, 0].set_title("MT Dipole")
        ax[0, 0].legend(["X", "Y", "Z", "Magnitude"])

        ax[0, 1].plot(np.array(fsw.times)*1e-9/3600, fsw.mt_cmd_history)
        ax[0, 1].plot(np.array(fsw.times)*1e-9/3600, np.linalg.norm(fsw.mt_cmd_history, axis=1))
        ax[0, 1].set_title("MT Dipole")
        ax[0, 1].legend(["X", "Y", "Z", "Magnitude"])
        ax[0, 1].set_ylim([-0.01, 0.01])

        ax[1, 0].plot(imuRec.times()*1e-9/3600, rotation_deg_data)
        ax[1, 0].plot(imuRec.times()*1e-9/3600, np.linalg.norm(rotation_deg_data, axis=1))
        ax[1, 0].set_title("rotation speed")
        ax[1, 0].legend(["x", "y", "z", "magnitude"])
        ax[1, 0].set_xlabel("time (hours)")
        ax[1, 0].set_ylabel("angular speed (deg/s)")
        # target speed deadband
        ax[1, 0].hlines(np.array([-0.002, 0.002])*180/np.pi, 0, max(imuRec.times())*1e-9/3600)
        # approximate maximum values that can be read by imu (+- 15 degrees)
        ax[1, 0].hlines([-15, 15], 0, max(imuRec.times())*1e-9/3600)

        ax[1, 1].plot(imuRec.times()*1e-9/3600, rotation_deg_data)
        ax[1, 1].plot(imuRec.times()*1e-9/3600, np.linalg.norm(rotation_deg_data, axis=1))
        ax[1, 1].set_title("rotation speed")
        ax[1, 1].legend(["x", "y", "z", "magnitude"])
        ax[1, 1].set_xlabel("time (hours)")
        ax[1, 1].set_ylabel("angular speed (deg/s)")
        # 0.002 is the controller threshold in rad/s
        ax[1, 1].hlines(np.array([-0.002, 0.002])*180/np.pi, 0, max(imuRec.times())*1e-9/3600)
        # ax[1, 1].set_ylim(np.array([-0.004, 0.004])*180/np.pi)


        ax[2, 0].plot(np.array(fsw.times)*1e-9/3600, fsw.omega_history)
        ax[2, 0].plot(np.array(fsw.times)*1e-9/3600, fsw.omega_norm_history)


        ax[2, 1].plot(sc_object_rec.times()*1e-9/3600, rotation_raw_data)


        fig, ax = plt.subplots(1, 2, figsize=(8, 6))
        skip_n = 0
        ax[0].plot(rotation_deg_data)
        ax[0].legend(["x", "y", "z"])

        color = plt.cm.magma(np.linspace(0, 1, len(mag_sensor_data[:,0]) -skip_n))
        limit = np.max(np.abs(rotation_deg_data))*1.5

        ax[1].remove()
        ax[1] = fig.add_subplot(1, 2, 2, projection="3d")
        ax[1].scatter(
                rotation_deg_data[skip_n:, 0],
                rotation_deg_data[skip_n:, 1],
                rotation_deg_data[skip_n:, 2], 
                c = color)
        ax[1].set_xlim([-limit, limit])
        ax[1].set_ylim([-limit, limit])
        ax[1].set_zlim([-limit, limit])
        ax[1].set_title("Rotation Vector")

    plt.show()






    if False:
        # Plot reaction wheel speeds
        fig, ax = plt.subplots(2, 1, figsize=(8, 8))
        ax[0].plot(rw_speed_rec.times()*1e-9, rw_speed_data)
        ax[0].set_title("Individual Speeds")
        ax[1].plot(rw_speed_rec.times()*1e-9, np.sum(np.abs(rw_speed_data), 1))
        ax[1].set_title("Total Speed")
        fig.suptitle("Reaction Wheels: Speed")

        if config["save_pdf"] == True:
            pdf_path = config["plot_basepath"] / "rw_speeds.pdf"
            plt.savefig(pdf_path, dpi=300)
            print(f"Plot saved as {pdf_path}")
        if config["save_png"] == True:
            png_path = config["plot_basepath"] / "rw_speeds.png"
            plt.savefig(png_path, dpi=600)
            print(f"Plot saved as {png_path}")

        # Plot battery power
        fig, ax = plt.subplots()
        ax.plot(battery_rec.times()*1e-9, batt_storage_data)
        ax.set_title("Battery: Stored Energy")

        if config["save_pdf"] == True:
            pdf_path = config["plot_basepath"] / "battery_storage.pdf"
            plt.savefig(pdf_path, dpi=300)
            print(f"Plot saved as {pdf_path}")
        if config["save_png"] == True:
            png_path = config["plot_basepath"] / "battery_storage.png"
            plt.savefig(png_path, dpi=600)
            print(f"Plot saved as {png_path}")


        # Plot solar panel power data
        fig, ax = plt.subplots()
        sp_time = solar_panel_recs[0].times()*1e-9
        for sp_data in sp_group_data:
            ax.plot(sp_time, sp_data)

        ax.plot(sp_time, np.sum(sp_group_data, axis=0))
        ax.set_title("Solar Power")

        if config["save_pdf"] == True:
            pdf_path = config["plot_basepath"] / "power_solar_panels.pdf"
            plt.savefig(pdf_path, dpi=300)
            print(f"Plot saved as {pdf_path}")
        if config["save_png"] == True:
            png_path = config["plot_basepath"] / "power_solar_panels.png"
            plt.savefig(png_path, dpi=600)
            print(f"Plot saved as {png_path}")

        # Plot reaction wheel power
        fig, ax = plt.subplots()
        rw_power_time = rw_power_recs[0].times()*1e-9
        for rw_power_data in rw_power_group_data:
            ax.plot(rw_power_time, rw_power_data)

        ax.plot(rw_power_time, np.sum(rw_power_group_data, axis=0))
        ax.set_title("Reaction Wheels: Power Usage")

        if config["save_pdf"] == True:
            pdf_path = config["plot_basepath"] / "power_reaction_wheels.pdf"
            plt.savefig(pdf_path, dpi=300)
            print(f"Plot saved as {pdf_path}")
        if config["save_png"] == True:
            png_path = config["plot_basepath"] / "power_reaction_wheels.png"
            plt.savefig(png_path, dpi=600)
            print(f"Plot saved as {png_path}")

        fig, ax = plt.subplots()
        # get true attitude error without sensor noise for graphing and filter comparison
        sigma_BN = np.array(stateRec.sigma_BN) # collects recorded spacecraft attitudes in MRP form. Extra rotation not necessary (as with filtered error in fsw) as it uses the same body frame as our system.
        q_scalar_first = [rbk.MRP2EP(attitude) for attitude in sigma_BN] # convert MRP's to scalar-first quaternions
        q_scalar_last = [quat.to_scalar_last(q) for q in q_scalar_first] # convert quaternions to scalar-last convention

        target_quats = np.repeat(fsw.target_history[:-1], int(fsw_update_time/dynamics_update_time), axis=0)

        # how about tracking the error in the flight software?
        tracking_error = [
            quat.error_angle(quat.quat_error(q_target, q)) 
            for (q_target, q) in zip(
                target_quats,
                q_scalar_last[:-1]
            )
        ]
        error_time = stateRec.times()*1e-9
        ax.plot(error_time[:-1], tracking_error)
        ax.set_title("Target Error")
        ax.set_ylabel("Error (degrees)")

        if config["save_pdf"] == True:
            pdf_path = config["plot_basepath"] / "target_error.pdf"
            plt.savefig(pdf_path, dpi=300)
            print(f"Plot saved as {pdf_path}")
        if config["save_png"] == True:
            png_path = config["plot_basepath"] / "target_error.png"
            plt.savefig(png_path, dpi=600)
            print(f"Plot saved as {png_path}")

        fig, ax = plt.subplots()
        ax.plot(imuRec.times()*1e-9/3600, rotation_data)
        ax.set_title("Angular rotation rates")
        ax.set_xlabel("Time (hours)")
        ax.set_ylabel("Rotational rate (r/s)")
        plt.show()


    if False:

        # get true attitude error without sensor noise for graphing and filter comparison
        sigma_BN = np.array(stateRec.sigma_BN) # collects recorded spacecraft attitudes in MRP form. Extra rotation not necessary (as with filtered error in fsw) as it uses the same body frame as our system.
        q_scalar_first = [rbk.MRP2EP(attitude) for attitude in sigma_BN] # convert MRP's to scalar-first quaternions
        q_scalar_last = [quat.to_scalar_last(q) for q in q_scalar_first] # convert quaternions to scalar-last convention
        
        # Target tracking mode requires special error calculations as we are dealing with step changes in target, which happens in fsw, not dynamics
        if config["guidance_mode"] is not None:
            tracking_error = [quat.error_angle(quat.quat_error(q_target, q)) for (q_target, q) in zip(fsw.target_history, q_scalar_last[::int(fsw_update_time/dynamics_update_time)])] # calculate step-errors for tracking mode
            error_true = np.repeat(tracking_error[:-1], fsw_update_time/dynamics_update_time, axis=0) # expand to match plotting times
            error_true = np.append(error_true, tracking_error[-1])
        else:
            error_true = [quat.error_angle(quat.quat_error(fsw.q_target, q)) for q in q_scalar_last] # calculate angle error (degrees) over simulation
        
        if(config["use_filter"]):
            error_angles_filter = [quat.error_angle(quaternion) for quaternion in fsw.error_filter[:-1]]
            error_expanded_filter = np.repeat(error_angles_filter, fsw_update_time/dynamics_update_time, axis=0)  # stretch all but last to match with times
            error_expanded_filter = np.append(error_expanded_filter, quat.error_angle(fsw.error_filter[-1])) # append final value
        else:
            error_expanded_filter = None
        
        plot_times = imuRec.times() * 1e-9
        if (config["control_mode"] in (ControlMode.RW_POINTING, ControlMode.RW_SLOW_ROTATE)):
            RW_plot_times = rw_speed_rec.times() * 1e-9 # dynamics process intervals

            time_axis = "seconds"
            plot_rw_speeds(RW_plot_times, rw_speed_rec.wheelSpeeds, numRW, config, error_true, error_expanded_filter)
            
            time_axis = "orbits"
            TAMvalues = mag_sensor_rec.tam_S
            plot_magfield(plot_times, TAMvalues, orbital_period, config, time_axis)


            fig, ax3 = plt.subplots(figsize=(8,4))
            ax3.plot(mtbLog.times() *1e-9, mtbLog.mtbNetTorque_B, 'r--', label='Torque')
            ax3.set_ylim([-1e-7, 1e-7])
            plt.title("MTB Net Torque")
            plt.legend()
        
            
            if config["save_pdf"] == True:
                pdf_path = config["plot_basepath"] / "mtb_torque_graph.pdf"
                plt.savefig(pdf_path, dpi=300)
                print(f"Plot saved as {pdf_path}")
            if config["save_png"] == True:
                png_path = config["plot_basepath"] / "mtb_torque_graph.png"
                plt.savefig(png_path, dpi=600)
                print(f"Plot saved as {png_path}")
            
            plt.show(block=False)
           
        else:
            time_axis = "orbits"
            TAMvalues = mag_sensor_rec.tam_S
            plot_magfield(plot_times, TAMvalues, orbital_period, config, time_axis)

            fig, ax2 = plt.subplots(figsize=(8,4))
            ax2.plot(plot_times, error_true, 'r--', label='True Error')
            
            plt.title("Error")
            plt.legend()
        
            
            if config["save_pdf"] == True:
                pdf_path = config["plot_basepath"] / "Err_graph.pdf"
                plt.savefig(pdf_path, dpi=300)
                print(f"Plot saved as {pdf_path}")
            if config["save_png"] == True:
                png_path = config["plot_basepath"] / "Err_graph.png"
                plt.savefig(png_path, dpi=600)
                print(f"Plot saved as {png_path}")
            
            plt.show(block=False)
     
            fig, ax3 = plt.subplots(figsize=(8,4))
            ax3.plot(mtbLog.times() *1e-9, mtbLog.mtbNetTorque_B, 'r--', label='Torque')
            ax3.set_ylim([-0.000005, 0.000005])
            plt.title("MTB Net Torque")
            plt.legend()
        
            
            if config["save_pdf"] == True:
                pdf_path = config["plot_basepath"] / "mtb_torque_graph.pdf"
                plt.savefig(pdf_path, dpi=300)
                print(f"Plot saved as {pdf_path}")
            if config["save_png"] == True:
                png_path = config["plot_basepath"] / "mtb_torque_graph.png"
                plt.savefig(png_path, dpi=600)
                print(f"Plot saved as {png_path}")
            
            plt.show(block=False)
         
        
        
        imuValues = imuRec.AngVelPlatform
        plot_imu(plot_times, imuValues, orbital_period, config, time_axis)


    # Finish with summary
    print(f"\nSimulation completed in {end-start:.2f} seconds\nSimulated time of flight: {config["sim_time"]} seconds")
    if config["control_mode"] in (ControlMode.RW_POINTING, ControlMode.MTB_POINTING):
        print(f"\nFinal target was: {fsw.q_target}")
        print(f"Angle from origin: {quat.error_angle(quat.quat_error(fsw.q_target, sat_q_init))}") # calculate orientation/angle change based on initial attitude
        if (config["sim_time"] >= config["error_time_check"]):
            print(f"Max error after {config["error_time_check"]} seconds:", max(error_true[int(config["error_time_check"] / config["dynamics_update_time"]):])) # this just checks for maximum error after a certain sim time (i.e. if large oscillations occur after steady-state should have been reached)
        print(f"Final error: {error_true[-1]:.4f} degrees")
    # if config["use_filter"] == True:
    #     print("\nFilter updates:", fsw.ticks)
    #     print("Filter corrections:", fsw.tracker_count)


