import numpy as np
import time
from Basilisk.simulation import spacecraft, starTracker, imuSensor, reactionWheelStateEffector, magneticFieldWMM, magnetometer, MtbEffector # import simulation related support
from Basilisk.utilities import SimulationBaseClass, macros, vizSupport, simIncludeGravBody, orbitalMotion, simIncludeRW, unitTestSupport # import general simulation support files
from Basilisk.architecture import messaging
from Basilisk import __path__
from Basilisk.utilities import RigidBodyKinematics as rbk
from Plotting_Functions import plot_rw_speeds, plot_magfield, plot_imu
from FlightSoftwareModule import FlightSoftware # self defined module to emulate flight software ADCS tasks
import Quaternions as quat
from pathlib import Path
from sys import exit
bskPath = __path__[0]

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
    scObject.hub.mHub = mass  # [kg], not sure this is required given inertial space, but should be realistic
    scObject.hub.IHubPntBc_B = J # assign OreSat inertia matrix
    initial_MRP = (np.array(init_rot_axis)/np.linalg.norm(np.array(init_rot_axis))) * np.tan(init_rot_angle*macros.D2R/4.0) # MRP set to customize initial inertial attitude
    scObject.hub.sigma_BNInit = initial_MRP
    scObject.hub.omega_BN_BInit = omega_init_rad
    
    ########################## ORBITAL ENVIRONMENT ############################
    
    # create gravitational bodies (Earth in this case, but might add moon later as well)
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # must be central for orbital motion
    mu_earth = earth.mu # get gravitational object's gravitational parameter
    gravFactory.addBodiesTo(scObject)  # automatically adds all created bodies
    
    # create the magnetic field
    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.ModelTag = "WMM" # World Magnetic Model
    magModule.dataPath = bskPath + '/supportData/MagneticField/'
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg('2025 June 27, 10:23:0.0 (UTC)')  # set epoch date/time message for WMM
    magModule.addSpacecraftToModel(scObject.scStateOutMsg) # add spacecraft to the magnetic field module so it can read the sc position messages
    sim.AddModelToTask("dynamicsTask", magModule) # add the magnetic field module to the simulation task
    magModule.epochInMsg.subscribeTo(epochMsg) # connect epoch messages
    
    # create orbit properties using classical orbit elements. Assuming perfectly circular orbit for now.
    oe = orbitalMotion.ClassicElements()
    oe.a = (415+6371) * 1e3 # semi-major axis  [meters] (altitude + earth's radius)
    oe.e = 0 # eccentricity
    oe.i = 50 * macros.D2R # inclination [rad]
    oe.Omega = 0 * macros.D2R  # RAAN or Longitude of the Ascending Node [rad]
    oe.omega = 0.0 * macros.D2R  # argument of periapsis [rad]
    oe.f = 10 * macros.D2R       # true anomaly [rad]
    
    rN, vN = orbitalMotion.elem2rv(mu_earth, oe)
    oe = orbitalMotion.rv2elem(mu_earth, rN, vN)  # this stores consistent initial orbit elements, fixes numerical errors, particulary with perfectly circular orbits. Consult ChatGPT for detailed explanation.
    orbital_period = 2*np.pi*np.sqrt(oe.a**3/mu_earth) # define orbital period for plotting
    orbital_inclination = oe.i # used for readable argument passing when defining FSW
    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # r_BN_N [m]
    scObject.hub.v_CN_NInit = vN  # v_BN_N [m/s]
    
    # Add spice object for planet rotation and ECEF coordinate simulation. Necessary for guidance algorithms.
    timeInitString = "2026-02-10T00:00:00Z"
    spiceObject = gravFactory.createSpiceInterface(bskPath + "/supportData/EphemerisData/", time=timeInitString, epochInMsg=True) # create SPICE object and point to ephemeris data
    spiceObject.addPlanetNames(["earth"])
    spiceObject.zeroBase = 'Earth' # centers the spice ephemeris data on Earth. Required, otherwise WMM becomes heliocentric, and has no effect on spacecraft in Earth orbit.
    
    earth.planetBodyInMsg.subscribeTo(spiceObject.planetStateOutMsgs[0])
    
    # Task ordering: SPICE before modules that consume its outputs. Need spice to run before spacecraft module.
    sim.AddModelToTask("dynamicsTask", spiceObject)
    sim.AddModelToTask("dynamicsTask", scObject)

    earthRec = spiceObject.planetStateOutMsgs[0].recorder() # used to record ECEF coordinates
    sim.AddModelToTask("dynamicsTask", earthRec)

    ############################### SENSORS ###################################
    
    # Create and configure a star tracker
    starTrackerSensor = starTracker.StarTracker()
    starTrackerSensor.ModelTag = "starTracker"
    
    # Define dcm_CB for star tracker orientation (body-to-case, star tracker on +x side)
    dcm_CB = np.array([[0.0, 0.0, -1.0], # x_B -> -z_C
                       [0.0, 1.0, 0.0],  # y_B -> y_C
                       [1.0, 0.0, 0.0]]) # z_B -> x_C
    starTrackerSensor.dcm_CB = dcm_CB
    
    starTrackerSensor.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    starTrackerRec = starTrackerSensor.sensorOutMsg.recorder(macros.sec2nano(fsw_update_time))
    
    # configure Star Tracker noise
    if config["use_filter"] == True:
        P = np.eye(3)*config["sigma_ST"]**2
        starTrackerSensor.PMatrix = P
        ST_bounds = 0.5
        starTrackerSensor.setWalkBounds([ST_bounds, ST_bounds, ST_bounds]) # bounds must be set in order to properly activate sensor noise
    
    sim.AddModelToTask("fswTask", starTrackerSensor) # Add sensor to flight software task
    sim.AddModelToTask("fswTask", starTrackerRec) # Add recording to task
    
    # Create and configure IMU sensor
    imu = imuSensor.ImuSensor()
    imu.ModelTag = "imu"
    imu.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    imuRec = imu.sensorOutMsg.recorder(macros.sec2nano(fsw_update_time)) # Record the output message
    
    # configure IMU noise
    if config["use_filter"] == True:
        P = np.eye(3)*config["sigma_gyro"]**2
        imu.PMatrixGyro = P
        imu_bounds = 5
        imu.setErrorBoundsGyro([imu_bounds, imu_bounds, imu_bounds]) # bounds must be set in order to properly activate sensor noise
    
    imu.UpdateState(0)  # Force IMU to process initial state, otherwise first value is set to zero for some reason
    sim.AddModelToTask("fswTask", imu) # Add sensor to flight software task
    sim.AddModelToTask("fswTask", imuRec) # Add recording to task
    
    # Create magnetometer sensor
    magSensor = magnetometer.Magnetometer()
    magSensor.ModelTag = "TAM_sensor" # Three-Axis Magnetometer
    magSensor.magInMsg.subscribeTo(magModule.envOutMsgs[0])
    magSensor.stateInMsg.subscribeTo(scObject.scStateOutMsg)
    magSensorRec = magSensor.tamDataOutMsg.recorder(macros.sec2nano(fsw_update_time))
    sim.AddModelToTask("fswTask", magSensor)
    sim.AddModelToTask("fswTask", magSensorRec) # Add recording to task
    
    ############################## EFFECTORS ##################################

    # Create reaction wheels
    # Define 4 reaction wheel unit vectors in a pyramid configuration (60 deg tilt from z-axis) 
    z = np.cos(60*np.pi/180) # wheel angle from z axis. Same for all wheels
    xy = np.cos(52.238756*np.pi/180) # wheel angle from x/y axis, sign varies by quadrant

                #  +x+y  +x-y  -x-y  -x+y  (motor positions in satellite quadrants. Each column represents one motor's torque components)
    G = np.array(([[xy,   xy,  -xy,  -xy],
                   [xy,  -xy,  -xy,   xy],
                   [-z,   -z,   -z,  -z]])) # Wheel moment/orientation matrix
    
    # rw_Inertia = 4.2946e-6      # [kg*m^2], moment of inertia about spin axis (old values from OreSat 0.5 wheels)
    rw_Inertia = 7.271e-6      # [kg*m^2], moment of inertia about spin axis
    
    maxSpeed = 11000.0 # ridiculous speed so our controller does the work. 100k effectively removes limit and allows fsw to limit manually.
    maxTorque = 100000.0 # only used when useMaxTorque = True. 100k effectively removes limit and allows fsw to limit manually.
    
    varRWModel = messaging.BalancedWheels # define wheel type as balanced (jitter is also an option)
    RWFactory = simIncludeRW.rwFactory() # create reaction wheel generator
    for i in range(len(G[0])): # create number of reaction wheels equal to wheels defined in G matrix
        axis = G[:,i]
        RWFactory.create(
            "custom",              # unique name
            axis,                  # spin axis
            Js=rw_Inertia,       # wheel inertia
            useMaxTorque=False,    # disable max torque check
            Omega_max=maxSpeed,    # max speed
            u_max = maxTorque,
            RWModel=varRWModel
        )
    numRW = RWFactory.getNumOfDevices() # counts the number of registered reaction wheels

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    RWFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)
    sim.AddModelToTask("dynamicsTask", rwStateEffector)
    
    # create magnetic torque bar (MTB) object
    mtbEff = MtbEffector.MtbEffector()
    mtbEff.ModelTag = "MtbEff"
    scObject.addDynamicEffector(mtbEff)
    sim.AddModelToTask("dynamicsTask", mtbEff)
    
    mtbConfigParams = messaging.MTBArrayConfigMsgPayload()
    mtbConfigParams.numMTB = 3
    mtbConfigParams.GtMatrix_B = [1., 0., 0., # expects single 1x(3*n) array
                                  0., 1., 0.,
                                  0., 0., 1.]
    # mtbConfigParams.maxMtbDipoles = [0.5e-2, 0.5e-2, 0.75e-2] # individual rod Dipole limits. Currently set to max continuous limit, not burst limit [A·m^2] (OLD LIMIT, WRONG VALUES???)
    mtbConfigParams.maxMtbDipoles = [1, 1, 1] # individual rod Dipole limits [A·m^2]

    mtbCfgMsg = messaging.MTBArrayConfigMsg().write(mtbConfigParams)
    
    mtbCmd = messaging.MTBCmdMsgPayload()
    mtbCmd.mtbDipoleCmds = [0.0] * mtbConfigParams.numMTB
    mtbCmdMsg = messaging.MTBCmdMsg().write(mtbCmd)
    
    # subscribe messages
    mtbEff.mtbParamsInMsg.subscribeTo(mtbCfgMsg)
    mtbEff.mtbCmdInMsg.subscribeTo(mtbCmdMsg)
    mtbEff.magInMsg.subscribeTo(magModule.envOutMsgs[0])  # from WMM module

    ############################ FLIGHT SOFTWARE ##############################
    # update config dict
    config.update({"G":G, "rw_Inertia":rw_Inertia, "orbital_period":orbital_period, "orbital_inclination":orbital_inclination})
    
    # Create flight software object and subscribe all sensors
    fsw = FlightSoftware(config) # Create flight software object. Model tag already defined in __init__ as flight_software
    fsw.starTrackerMsgIn.subscribeTo(starTrackerSensor.sensorOutMsg) # subscribe to star tracker messages
    fsw.imuMsgIn.subscribeTo(imu.sensorOutMsg) # subscribe to IMU messages
    fsw.rwSpeedMsgIn.subscribeTo(rwStateEffector.rwSpeedOutMsg) # subscribe fsw reaction wheel speed input to reaction wheel output
    fsw.magMsgIn.subscribeTo(magSensor.tamDataOutMsg) # subscribe fsw to magenotometer readings
    fsw.scStateIn.subscribeTo(scObject.scStateOutMsg) # subscribe fsw to spacecraft state (positioning) for GPS emulation
    fsw.earthStateInMsg.subscribeTo(spiceObject.planetStateOutMsgs[0]) # subscribe fsw to earth SPICE data, necessary to extract J20002PFix transformation matrix
    fsw.set_time_zero_from_iso_utc(timeInitString) # initialize ephemeris start time for GPS timestamp emulation
    sim.AddModelToTask("fswTask", fsw)
    
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(fsw.rwMotorTorqueOutMsg) # subscribe reaction wheel command input to flight software control output
    mtbEff.mtbCmdInMsg.subscribeTo(fsw.magTorqueOutMsg)  # subscribe magnetorquer command input to flight software control output
    
    # determine initial pointing vector for relative target calculations
    sat_q_init = quat.axis_angle_to_quaternion(init_rot_axis, init_rot_angle) # account for any rotations of the satellite it self at sim initialization
    if fsw.pointing == "ST":
        q_init = quat.quat_mult(quat.axis_angle_to_quaternion([0,1,0], 90), sat_q_init)
    elif fsw.pointing == "SC":
        q_init = sat_q_init
    elif fsw.pointing == "CFC":
        q_init = quat.quat_mult(quat.axis_angle_to_quaternion([0,1,0], 180), sat_q_init)
    else:
        print("ERROR: Invalid pointing reference selected!")
        exit()

    q_rot = quat.axis_angle_to_quaternion(sat_rot_axis, sat_rot_angle)
    fsw.update_target(quat.quat_mult(q_rot, q_init))
    
    # print(f"\nSatellite view device is \"{fsw.pointing}\" with initial reference: {q_init}")
    # print(f"Satellite initial pointing target: {fsw.q_target}\n")
    
    ############################## SIMULATION #################################
    # log reaction wheel behavior
    rwSpeedLog = rwStateEffector.rwSpeedOutMsg.recorder()
    sim.AddModelToTask("dynamicsTask", rwSpeedLog)

    # add spacecraft state recording in order to read attitudes for plotting
    stateRec = scObject.scStateOutMsg.recorder(macros.sec2nano(dynamics_update_time)) # create dynamics recorder
    sim.AddModelToTask("dynamicsTask", stateRec) # add recorder to dynamics simulation
    
    basePath = r"C:\Users\benne\OneDrive\Master's Thesis\Code\Viz_Archive"
    if viz_filename:
        fileName = basePath + rf"\{viz_filename}"
    else:
        fileName = __file__
    
    current_dir = Path(__file__).parent.resolve() # find current working directory such that any system running code directly from git can use the simplified model
    model_file_path = current_dir / config["sat_3D_file"]

    viz = vizSupport.enableUnityVisualization(sim, "dynamicsTask", scObject, saveFile=fileName, liveStream=False, # let Vizard visualize data
                                              rwEffectorList=rwStateEffector) # add reaction wheel list to visualization
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
    
    # get true attitude error without sensor noise for graphing and filter comparison
    sigma_BN = np.array(stateRec.sigma_BN) # collects recorded spacecraft attitudes in MRP form. Extra rotation not necessary (as with filtered error in fsw) as it uses the same body frame as our system.
    q_scalar_first = [rbk.MRP2EP(attitude) for attitude in sigma_BN] # convert MRP's to scalar-first quaternions
    q_scalar_last = [quat.to_scalar_last(q) for q in q_scalar_first] # convert quaternions to scalar-last convention
    
    # Target tracking mode requires special error calculations as we are dealing with step changes in target, which happens in fsw, not dynamics
    if config["tracking_mode"] is not None:
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
    if ("RW" in config["control_mode"]):
        RW_plot_times = rwSpeedLog.times() * 1e-9 # dynamics process intervals
        plot_rw_speeds(RW_plot_times, rwSpeedLog.wheelSpeeds, numRW, config, error_true, error_expanded_filter)
        time_axis = "seconds"
    else:
        time_axis = "orbits"
        TAMvalues = magSensorRec.tam_S
        plot_magfield(plot_times, TAMvalues, orbital_period, config, time_axis)
    
    imuValues = imuRec.AngVelPlatform
    plot_imu(plot_times, imuValues, orbital_period, config, time_axis)
    # plot_imu(plot_times, imuValues, orbital_period, config, time_axis, error_true)
    
    print(f"\nSimulation completed in {end-start:.2f} seconds\nSimulated time of flight: {config["sim_time"]} seconds")
    if config["control_mode"] == "RW_POINTING" or config["control_mode"] == "MTB_POINTING":
        print(f"\nFinal target was: {fsw.q_target}")
        print(f"Angle from origin: {quat.error_angle(quat.quat_error(fsw.q_target, sat_q_init))}") # calculate orientation/angle change based on initial attitude
        if (config["sim_time"] >= config["error_time_check"]):
            print(f"Max error after {config["error_time_check"]} seconds:", max(error_true[int(config["error_time_check"] / config["dynamics_update_time"]):])) # this just checks for maximum error after a certain sim time (i.e. if large oscillations occur after steady-state should have been reached)
        print(f"Final error: {error_true[-1]:.3f} degrees")
    if config["use_filter"] == True:
        print("\nFilter updates:", fsw.ticks)
        print("Filter corrections:", fsw.tracker_count)
        
if __name__ == "__main__":
    # select satellite model attributes
    # satellite = "OreSat1"
    satellite = "SENTINEL"
    
    # select 3D model file
    if satellite == "SENTINEL":
        sat_3D_file = "3U_Simplified_Model.obj"
        viz_scaling = 5 # 3D-model scaling factor
    
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
    else:
        sat_3D_file = "OreSat_Simplified_Model.obj"
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
                  
    viz_filename = None # sim visualization savename
    print_states = False # print states in flight software
    save_pdf = True # save plots as PDF's to target folder
    save_png = False # save plots as PNG's to target folder
    plot_basepath = Path(r"C:\Users\benne\OneDrive\Master's Thesis\Basilisk_Output") # path to which graphs should be saved
    use_filter = True # whether to use perfect state information or simulate with sensor noise and state estimation (MEKF)
    error_time_check = 100 # time after which maximum error is considered for evaluation
    
    # sensor noise parameters
    # Keep two separate parameters
    # sigma_gyro_density = 0.014 * D2R (for MEKF Q_matrix)
    # sigma_gyro_rms = 0.1 * D2R (for Basilisk imu.PMatrixGyro)
    
    # sigma_gyro = 0.1 * macros.D2R # instantaneous white noise (datasheet gives value in degrees, convert to radians) (not sure which to use)
    sigma_gyro = 0.014 * macros.D2R # instantaneous white noise (datasheet gives value in degrees, convert to radians) (not sure which to use)
    sigma_bias = 1e-5 # slow random bias drift (random walk)
    P_b0 = 1 * macros.D2R # [rad/s] initial gyro uncertainty
    
    sigma_ST = 2.4e-6 # [rad] measurement noise (instantaneous orientation error)
    P_ST_0 = 8.7e-7 # [rad^2] initial star tracker attitude uncertainty
    ST_update_rate = 1.1 # defined in seconds
    
    # initial satellite states
    init_rot_axis = [1, 0, 0]# this vector cannot be all zeros or quat.axis_angle_to_quaternion will return nan! 
    init_rot_angle = 0
    
    # omega_init_rpm = np.array([3.0, 0.4, 0.7])  # initial spin velocties [RPM]
    omega_init_rpm = np.array([0.0, 0.0, 0.0])  # initial spin velocties [RPM]
    omega_init_rad = omega_init_rpm * 2*np.pi/60  # convert RPM to rad/s
    
    # command rotations relative to initial orientation
    sat_rot_axis = [1, 1, 0]
    sat_rot_angle = 180
    
    # Select the spacecraft pointing reference (which axis/sensor defines boresight) and control modes:    
    pointing_reference = "SC" # Modes are ST (Star Tracker, +x on body), SC (Selfie Camera, +z on body), and CFC (Cirrus Flux Camera, -z on body)  
    control_mode = "MTB_POINTING" # Valid modes are DETUMBLE, RW_POINTING, MTB_POINTING, THERMAL_SPIN, ORBITS. ORBITS is for long-duration visualization without controls

    
    # Track specified target on Earth's surface or nadir vector. Both with +x axis ram-facing.
    tracking_mode = None
    # tracking_mode = "TARGET" # Valid modes are TARGET, NADIR, MAX_DRAG, MIN_DRAG, or None. Max and min drag modes face +x or +z into ram direction respectively.
    use_skyfield = True
    # KSAT coordinates
    # target_lat = 78.231500
    # target_lon = 15.411100
    # target_height = 488 # [m]
    
    # ESI headquarters coordinates
    target_lat = 39.608251
    target_lon = -104.895788
    target_height = 1716 # [m]
        
    if ("RW" in control_mode): # realistic RW sim setup
        sim_time = 800
        dynamics_update_time = .1
        fsw_update_time = .1
        if (fsw_update_time > 2): # give user warning about unrealistic time steps so THEY DON'T WASTE TIME
            print("\nWARNING: FSW update time too large for stable convergence with reaction wheels\nExiting sim")
            exit()
    elif control_mode == "ORBITS":
        sim_time = 12000
        dynamics_update_time = 10
        fsw_update_time = 10
        use_filter = False
        omega_init_rad = np.array([0.0, 0.0, 0.0]) # ensure no excessive spinning
    else: # realistic MTB sim setup
        sim_time = 3000
        dynamics_update_time = .1
        fsw_update_time = .1
    
    print(f"Satellite: {satellite}")
    print(f"Mission Mode: {control_mode}")
    print(f"View Device: {pointing_reference}")
    print(f"Tracking Mode: {tracking_mode}\n")
    
    config = {"J":J, "mass":mass, "init_rot_axis":init_rot_axis, "init_rot_angle":init_rot_angle, "omega_init_rpm":omega_init_rpm, "omega_init_rad":omega_init_rad,
              "satellite":satellite, "sat_rot_axis":sat_rot_axis, "sat_rot_angle":sat_rot_angle, "pointing_reference":pointing_reference, "control_mode":control_mode,
              "sim_time":sim_time, "dynamics_update_time":dynamics_update_time, "fsw_update_time":fsw_update_time, "viz_filename":viz_filename, "print_states":print_states,
              "save_pdf":save_pdf, "save_png":save_png, "plot_basepath":plot_basepath, "use_filter":use_filter, "sigma_gyro":sigma_gyro, "sigma_bias":sigma_bias, "P_b0":P_b0,
              "sigma_ST":sigma_ST, "P_ST_0":P_ST_0, "ST_update_rate":ST_update_rate, "error_time_check":error_time_check, "tracking_mode":tracking_mode, 
              "target_lat":target_lat, "target_lon":target_lon, "target_height":target_height, "sat_3D_file":sat_3D_file, "viz_scaling":viz_scaling, "use_skyfield":use_skyfield}
    
    sim_main(config)