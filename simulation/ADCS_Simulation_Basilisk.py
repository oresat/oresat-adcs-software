import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
from Basilisk.simulation import spacecraft, starTracker, imuSensor, reactionWheelStateEffector, magneticFieldWMM, magnetometer, MtbEffector # import simulation related support
from Basilisk.utilities import SimulationBaseClass, macros, vizSupport, simIncludeGravBody, orbitalMotion, simIncludeRW, unitTestSupport # import general simulation support files
from Basilisk.architecture import messaging
from Basilisk import __path__
from Plotting_Functions import plot_rw_speeds, plot_magfield
from FlightSoftwareModule import FlightSoftware # self defined module to emulate flight software ADCS tasks
from scipy.spatial.transform import Rotation as R # to create nadir pointing quaternion
import Quaternions as quat
import sys
bskPath = __path__[0]

def sim_main(simTime, J, mass, dynamics_update_time, fsw_update_time, viz_filename, init_rot_axis, 
             init_rot_angle, omega_init_rad, sat_rot_axis, sat_rot_angle, pointing_reference, print_states, control_mode):
    """
    Gets all satellite states (attitude quaternion, omega)
    
    Parameters:
    simTime: time over which simulation runs
    J: inertia matrix of spacecraft
    dynamics_update_time: update time of the dynamics simulation
    fsw_update_time: update time of the flight software module
    
    Returns:
    A sick simulation
    """
    
    # simulation variables
    dynamics_update_time = dynamics_update_time # seconds
    fsw_update_time = fsw_update_time # temporarily REALLY small to make the system respond as intended
    
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
    scObject.ModelTag = "OreSat" # name object
    scObject.hub.mHub = mass  # [kg], not sure this is required given inertial space, but should be realistic
    scObject.hub.IHubPntBc_B = J # assign OreSat inertia matrix
    initial_MRP = (np.array(init_rot_axis)/np.linalg.norm(np.array(init_rot_axis))) * np.tan(init_rot_angle*macros.D2R/4.0) # MRP set to customize initial inertial attitude
    scObject.hub.sigma_BNInit = initial_MRP
    scObject.hub.omega_BN_BInit = omega_init_rad
    sim.AddModelToTask("dynamicsTask", scObject) # add spacecraft to the dynamics simulation
    
    ########################## ORBITAL ENVIRONMENT ############################
    
    # create gravitational bodies (earth in this case, but might add moon later as well)
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
    oe.i = 0 * macros.D2R # inclination [rad]
    oe.Omega = 0.0 * macros.D2R  # RAAN or Longitude of the Ascending Node [rad]
    oe.omega = 0.0 * macros.D2R  # argument of periapsis [rad]
    oe.f = 90 * macros.D2R       # true anomaly [rad]
    
    rN, vN = orbitalMotion.elem2rv(mu_earth, oe)
    oe = orbitalMotion.rv2elem(mu_earth, rN, vN)  # this stores consistent initial orbit elements, fixes numerical errors, particulary with perfectly circular orbits. Consult ChatGPT for detailed explanation.
    orbital_period = 2*np.pi*np.sqrt(oe.a**3/mu_earth) # define orbital period for plotting
    
    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # r_BN_N [m]
    scObject.hub.v_CN_NInit = vN  # v_BN_N [m/s]
    
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
    sim.AddModelToTask("fswTask", starTrackerSensor) # Add sensor to flight software task
    sim.AddModelToTask("fswTask", starTrackerRec) # Add recording to task
    
    # Create and configure IMU sensor
    imu = imuSensor.ImuSensor()
    imu.ModelTag = "imu"
    imu.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    imuRec = imu.sensorOutMsg.recorder(macros.sec2nano(fsw_update_time)) # Record the output message
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
    
    wheelInertia = 4.2946e-6      # [kg*m^2], moment of inertia about spin axis
    maxSpeed = 11000.0 # ridiculous speed so our controller does the work. 100k effectively removes limit, and allows fsw to limit manually.
    maxTorque = 100000.0 # only used when useMaxTorque = True. 100k effectively removes limit, and allows fsw to limit manually.
    
    varRWModel = messaging.BalancedWheels # define wheel type as balanced (jitter is also an option)
    RWFactory = simIncludeRW.rwFactory() # create reaction wheel generator
    for i in range(len(G[0])): # create number of reaction wheels equal to wheels defined in G matrix
        axis = G[:,i]
        RWFactory.create(
            "custom",              # unique name
            axis,                  # spin axis
            Js=wheelInertia,       # wheel inertia
            useMaxTorque=False,    # disable max torque check
            Omega_max=maxSpeed,    # max speed
            u_max = maxTorque,
            RWModel=varRWModel
        )
        
    numRW = RWFactory.getNumOfDevices()
    print(f"\nFound {numRW} reaction wheels in satellite")

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    RWFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)
    sim.AddModelToTask("dynamicsTask", rwStateEffector)
    
    
    # create magnetic torque bar object
    mtb = MtbEffector.MtbEffector()
    mtb.ModelTag = "MTB"
    mtbConfigParams = messaging.MTBArrayConfigMsgPayload()
    mtbConfigParams.numMTB = 3
    
    # row major torque bar alignments
    mtbConfigParams.GtMatrix_B = [1., 0., 0.,
                                  0., 1., 0.,
                                  0., 0., 1.]
    maxDipole = 0.1
    # scObject.addDynamicEffector(mtb)
    # sim.AddModelToTask("dynamicsTask", mtb)

    ############################ FLIGHT SOFTWARE ##############################

    # Create flight software object and subscribe all sensors
    fsw = FlightSoftware(G, fsw_update_time, wheelInertia, J, pointing_reference, print_states, control_mode) # Create flight software object. Model tag already defined in __init__ as flight_software
    fsw.starTrackerMsgIn.subscribeTo(starTrackerSensor.sensorOutMsg) # subscribe to star tracker messages
    fsw.imuMsgIn.subscribeTo(imu.sensorOutMsg) # subscribe to IMU messages
    fsw.rwSpeedMsgIn.subscribeTo(rwStateEffector.rwSpeedOutMsg) # subscribe fsw reaction wheel speed input to reaction wheel output
    fsw.magMsgIn.subscribeTo(magSensor.tamDataOutMsg)
    sim.AddModelToTask("fswTask", fsw)
    
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(fsw.rwMotorTorqueOutMsg) # subscribe reaction wheel input to flight software control output
    
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
        sys.exit()
    
    q_rot = quat.axis_angle_to_quaternion(sat_rot_axis, sat_rot_angle)
    fsw.q_target = quat.quat_mult(q_rot, q_init) # Probably shouldn't be applying hemisphere check to this operation! Removed for now, check logic.
    
    print(f"Satellite view device is \"{fsw.pointing}\" with initial reference: {q_init}")
    print(f"Satellite initial pointing target: {fsw.q_target}\n")
    
    ############################## SIMULATION #################################
    
    rwSpeedLog = rwStateEffector.rwSpeedOutMsg.recorder()
    sim.AddModelToTask("dynamicsTask", rwSpeedLog)

    # add simulation recording
    stateRec = scObject.scStateOutMsg.recorder(macros.sec2nano(dynamics_update_time)) # create recorder of dynamics
    sim.AddModelToTask("dynamicsTask", stateRec) # add recorder to dynamics simulation
    
    basePath = r"C:\Users\benne\OneDrive\Master's Thesis\Code\Viz_Archive"
    if viz_filename:
        fileName = basePath + rf"\{viz_filename}"
    else:
        fileName = __file__
    
    viz = vizSupport.enableUnityVisualization(sim, "dynamicsTask", scObject, saveFile=fileName, liveStream=False, # let Vizard visualize data
                                              rwEffectorList=rwStateEffector) # add reaction wheel list to visualization
    vizSupport.setActuatorGuiSetting(viz, viewRWPanel=True, viewRWHUD=True)
    vizSupport.createCustomModel(viz,
                                 modelPath=r"C:\Users\benne\OneDrive\Master's Thesis\Code\OreSat_Simplified_Model.obj",
                                 scale=[-7, 7, 7], # scale model and mirror on x-axis
                                 rotation=[0,np.pi/2,np.pi/2]) # rotate to properly align body axes with simulation axes
    
    # simulate:
    sim.InitializeSimulation() # initialize simulation
    sim.ConfigureStopTime(macros.sec2nano(simTime)) # configure a simulation stop time
    print("\nSimulation setup complete\nBeginning simulation\n")
    start = time.time()
    sim.ExecuteSimulation() # execute simulation
    end = time.time()
    
    plot_times = rwSpeedLog.times() * 1e-9
    error_angles = [quat.error_angle(quaternion) for quaternion in fsw.error[:-1]]
    error_expanded = np.repeat(error_angles, fsw_update_time/dynamics_update_time, axis=0)  # stretch all but last to match with times
    error_expanded = np.append(error_expanded, quat.error_angle(fsw.error[-1])) # append final value
    plot_rw_speeds(plot_times, rwSpeedLog.wheelSpeeds, numRW, error_expanded)
    
    TAMvalues = magSensorRec.tam_S
    TAMtimes = magSensorRec.times()*1e-9
    plot_magfield(TAMtimes, TAMvalues, orbital_period, "orbits")
    
    print(f"\nSimulation completed in {end-start} seconds")
    print(f"\nFinal target was: {fsw.q_target}")
    print(f"Angle from origin: {quat.error_angle(quat.quat_error(fsw.q_target, q_init))}")
    print(f"Final error: {quat.error_angle(fsw.error[-1]):.3f} degrees")
    
if __name__ == "__main__":
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
    print_states = False
    
    sim_time = 6000
    dynamics_update_time = 60
    fsw_update_time = 60
    
    # initial satellite states
    init_rot_axis = [0, 1, 1]# this vector cannot be all zeros or quat.axis_angle_to_quaternion will return nan! 
    init_rot_angle = 0
    temp = quat.axis_angle_to_quaternion(init_rot_axis, init_rot_angle)
    omega_init_rpm = np.array([0.0, 0.0, 0.0])  # initial spin velocties [RPM]
    omega_init_rad = omega_init_rpm * 2*np.pi/60  # convert RPM to rad/s
    
    # command rotations relative to initial orientation
    sat_rot_axis = [1, 1.5, 0]
    sat_rot_angle = 165
    
    # Select the spacecraft pointing reference (which axis/sensor defines boresight):
    # Modes are ST (Star Tracker, +x on body), SC (Selfie Camera, +z on body), and CFC (Cirrus Flux Camera, -z on body)  
    pointing_reference = "ST"
    control_mode = "MAG"
    
    sim_main(sim_time, J, mass, dynamics_update_time, fsw_update_time, viz_filename, init_rot_axis, init_rot_angle, omega_init_rad, sat_rot_axis, sat_rot_angle, pointing_reference, print_states, control_mode) # call and run simulation
