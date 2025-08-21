import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
from Basilisk.simulation import spacecraft, starTracker, imuSensor, reactionWheelStateEffector # import simulation related support
from Basilisk.utilities import SimulationBaseClass, macros, vizSupport, simIncludeGravBody, orbitalMotion, simIncludeRW # import general simulation support files
from Basilisk.architecture import messaging
from FlightSoftwareModule import FlightSoftware # self defined module to emulate flight software ADCS tasks
from scipy.spatial.transform import Rotation as R # to create nadir pointing quaternion
import Quaternions as quat

def getLineColor(idx, maxNum):
    """pick a nicer color pattern to plot 3 vector components"""
    values = list(range(0, maxNum + 2))
    colorMap = plt.get_cmap('viridis') # select color from https://matplotlib.org/stable/users/explain/colors/colormaps.html gist_ncar, nipy_spectral, inferno, viridis
    cNorm = colors.Normalize(vmin=0, vmax=values[-1])
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=colorMap)
    return scalarMap.to_rgba(values[idx + 1])

# def plot_rw_speeds(timeData, dataOmegaRW, numRW):
#     """Plot the RW spin rates."""
#     plt.figure(4)
#     for idx in range(numRW):
#         plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
#                  color=getLineColor(idx, numRW),
#                  label=r'$\Omega_{' + str(idx) + '}$')
#     plt.grid(True)
#     # plt.legend()
#     plt.legend(loc='upper left')
#     plt.xlabel('Time')
#     plt.ylabel('RW Speed (RPM) ')

def plot_rw_speeds(timeData, dataOmegaRW, numRW, errorArray=None):
    """Plot the RW spin rates with optional error curve on right axis."""
    fig, ax1 = plt.subplots(figsize=(8,5))

    # --- Reaction wheel speeds ---
    for idx in range(numRW):
        ax1.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx) + '}$')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('RW Speed (RPM)')
    ax1.grid(True)

    # --- Optional error line ---
    if errorArray is not None:
        ax2 = ax1.twinx()  # create second y-axis
        ax2.plot(timeData, errorArray, 'r--', label='Error')
        ax2.set_ylabel('Error')
        # Add legend for error separately
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    else:
        plt.legend()
    plt.title("Reaction Wheel Speeds and Error")
    plt.show()

def get_nadir_pointing_quaternion(r_BN_N, body_axis=np.array([0, 0, 1])):
    """
    Computes the quaternion that aligns a body axis (default: +Z) toward Earth.
    
    Parameters:
    r_BN_N (numpy.ndarray): Position vector of satellite in inertial frame (3,)
    body_axis (numpy.ndarray): Desired body axis to point toward Earth (default +Z)
    
    Returns:
    np.ndarray: scalar-last quaternion [x, y, z, s]
    """
    # Normalize body and inertial vectors
    r_hat = -r_BN_N / np.linalg.norm(r_BN_N)  # Nadir direction in inertial frame
    b_hat = body_axis / np.linalg.norm(body_axis)  # Axis in body frame

    # Compute rotation axis (cross product) and angle
    cross = np.cross(b_hat, r_hat)
    dot = np.dot(b_hat, r_hat)
    norm_cross = np.linalg.norm(cross)

    if norm_cross < 1e-8:  # vectors are nearly aligned or opposite
        if dot > 0:
            return np.array([0, 0, 0, 1])  # identity quaternion
        else:
            # 180° rotation around any axis perpendicular to b_hat
            axis = np.array([1, 0, 0]) if abs(b_hat[0]) < 0.9 else np.array([0, 1, 0])
            perp = np.cross(b_hat, axis)
            perp /= np.linalg.norm(perp)
            q_vec = perp * np.sin(np.pi/2)
            q_scalar = np.cos(np.pi/2)
            return np.concatenate((q_vec, [q_scalar]))

    axis = cross / norm_cross
    angle = np.arccos(np.clip(dot, -1.0, 1.0))
    rot = R.from_rotvec(axis * angle)
    q = rot.as_quat()  # returns [x, y, z, w] (scalar-last)
    return q

def sim_main(simTime, J, mass, dynamics_update_time, fsw_update_time, viz_filename):
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
    omega_init_rpm = np.array([0.0, 0.0, 0.0])  # intial spin velocties [RPM]
    omega_init_rad = omega_init_rpm * 2*np.pi/60  # convert RPM to rad/s
    # omega_init_rad = np.array([3.9479893375181184e-05, -2.6694436384127718e-11, -2.247383471180911e-11])
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
    
    # create spacecraft object
    scObject = spacecraft.Spacecraft() # initialize object
    scObject.ModelTag = "OreSat" # name object
    scObject.hub.mHub = mass  # [kg], not sure this is required given inertial space, but should be realistic
    scObject.hub.IHubPntBc_B = J # assign OreSat inertia matrix
    scObject.hub.omega_BN_BInit = omega_init_rad
    sim.AddModelToTask("dynamicsTask", scObject) # add spacecraft to the dynamics simulation
    print(dir(scObject.hub))

    # create gravitational bodies (earth in this case, but might add moon later as well)
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # must be central for orbital motion
    mu_earth = earth.mu # get gravitational object's gravitational parameter
    gravFactory.addBodiesTo(scObject)  # automatically adds all created bodies
    
    # create orbit properties using classical orbit elements. Assuming perfectly circular orbit for now.
    oe = orbitalMotion.ClassicElements()
    oe.a = (460+6371) * 1e3 # semi-major axis  [meters] (altitude + earth's radius)
    oe.e = 0 # eccentricity
    oe.i = 80 * macros.D2R # inclination [rad]
    oe.Omega = 40.0 * macros.D2R  # RAAN or Longitude of the Ascending Node [rad]
    oe.omega = 30.0 * macros.D2R  # argument of periapsis [rad]
    oe.f = 0 * macros.D2R       # true anomaly [rad]
    
    rN, vN = orbitalMotion.elem2rv(mu_earth, oe)
    oe = orbitalMotion.rv2elem(mu_earth, rN, vN)      # this stores consistent initial orbit elements, fixes numerical errors, particulary with perfectly circular orbits. Consult ChatGPT for detailed explanation.
    
    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # r_BN_N [m]
    scObject.hub.v_CN_NInit = vN  # v_BN_N [m/s]
    
    # Create and configure a star tracker
    starTrackerSensor = starTracker.StarTracker()
    starTrackerSensor.ModelTag = "starTracker"
    
    # Define dcm_CB for star tracker orientation (body-to-case, star tracker on +x side)
    dcm_CB = np.array([[0.0, 0.0, -1.0], # x_B -> -y_C
                       [0.0, 1.0, 0.0],  # y_B -> y_C
                       [1.0, 0.0, 0.0]]) # z_B -> x_C
    # dcm_CB = np.array([[1.0, 0.0, 0.0], # x_B -> x_C
    #                    [0.0, 0.0, 1.0],  # y_B -> z_C
    #                    [0.0, -1.0, 0.0]]) # z_B -> x_C
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
    
    # Create reaction wheels
    # Define 4 reaction wheel unit vectors in a pyramid configuration (60 deg tilt from z-axis) 
    z = np.cos(60*np.pi/180) # wheel angle from z axis
    xy = np.cos(52.238756*np.pi/180) # wheel angle from x/y axis, sign varies by quadrant
                #  +x+y  +x-y  -x-y  -x+y  (motor locations)
    G = np.array(([[xy,   xy,  -xy,  -xy],
                   [xy,  -xy,  -xy,   xy],
                   [-z,   -z,   -z,  -z]])) # Wheel moment/orientation matrix
    
    wheelInertia = 4.2946e-6      # [kg*m^2], moment of inertia about spin axis
    maxSpeed = 10000.0 # ridiculous speed so our controller does the work. 100k effectively removes limit, and allows fsw to limit manually.
    maxTorque = 100000.0 # only used when useMaxTorque = True. 100k effectively removes limit, and allows fsw to limit manually.
    
    varRWModel = messaging.BalancedWheels # define wheel type as balanced (jitter is also an option)
    RWFactory = simIncludeRW.rwFactory() # create reaction wheel generator
    for i in range(4):
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
    sim.AddModelToTask("dynamicsTask", rwStateEffector, None, 2)

    # Create flight software object
    fsw = FlightSoftware(G, fsw_update_time, wheelInertia, J) # Create flight software object. Model tag already defined in __init__ as flight_software
    fsw.starTrackerMsgIn.subscribeTo(starTrackerSensor.sensorOutMsg) # subscribe to star tracker messages
    fsw.imuMsgIn.subscribeTo(imu.sensorOutMsg) # subscribe to IMU messages
    fsw.rwSpeedMsgIn.subscribeTo(rwStateEffector.rwSpeedOutMsg) # subscribe fsw reaction wheel speed input to reaction wheel output
    sim.AddModelToTask("fswTask", fsw)
    
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(fsw.rwMotorTorqueOutMsg) # subscribe reaction wheel input to flight software control output
    
    q_init = quat.axis_angle_to_quaternion([0,1,0], 90) # same orientation as initial state
    # fsw.q_target = get_nadir_pointing_quaternion(rN)
    # # fsw.q_target = [0.0, 0.6,  0.0,          0.8]
    # fsw.q_target = [0.61850101, -0.60304099,  0.0,          0.50378375]
    # fsw.q_target = [0.0,        0.4358898943540673, 0.0,         0.9]
    # # fsw.q_target = [0.0,        0.31224989991991997, 0.0,       0.95]
    # # fsw.q_target = [0.5, 0.5, 0.5, 0.5]
    
    axis = [0,1,0]
    q_rot = quat.axis_angle_to_quaternion(axis, 90)
    fsw.q_target = quat.quat_mult(q_rot, q_init)
    
    # q_last = [0.019739947837826483, -1.3350097999298427e-08, -1.1241915620630039e-08, 0.9998051482460769]
    # fsw.q_target = quat.quat_mult(q_last, q_init)
    
    rwSpeedLog = rwStateEffector.rwSpeedOutMsg.recorder()
    sim.AddModelToTask("dynamicsTask", rwSpeedLog)
    # print("Is RW input message linked?", rwStateEffector.rwMotorCmdInMsg.isLinked(), "\n")


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
    
    print("\nSimulation setup complete\nBeginning simulation\n")
    
    angle_quat = quat.quat_error(fsw.q_target, q_init)
    error_angle_degrees = quat.error_angle(angle_quat) # get minimum error angle (in degrees)
    print("Targeting angle change of", error_angle_degrees, "deg\n")
    
    # simulate:
    sim.InitializeSimulation() # initialize simulation
    sim.ConfigureStopTime(macros.sec2nano(simTime)) # configure a simulation stop time
    start = time.time()
    
    sim.ExecuteSimulation() # execute simulation
    end = time.time()
    
    print(f"\nSimulation completed in {end-start} seconds")
    print(f"Vizard visualization saved to: {fileName}")
    
    plot_times = rwSpeedLog.times() * 1e-9
    
    error_angles = [quat.error_angle(quaternion) for quaternion in fsw.error[:-1]]
    error_expanded = np.repeat(error_angles, 10, axis=0)  # stretch all but last to match with times
    error_expanded = np.append(error_expanded, quat.error_angle(fsw.error[-1])) # append final value
    # error_expanded = None
    plot_rw_speeds(plot_times, rwSpeedLog.wheelSpeeds, numRW, error_expanded)
    print(fsw.q_target)
    
if __name__ == "__main__":
    # Jxx = 0.01537002
    # Jxy = 0.00001166
    # Jxz = 0.00022389
    # Jyx = 0.00001166
    # Jyy = 0.01449756
    # Jyz = 0.0000318
    # Jzx = 0.00022389
    # Jzy = 0.0000318
    # Jzz = 0.00576094
    
    Jxx = 0.01650237
    Jxy = 0.00000711
    Jxz = 0.00004547
    Jyx = 7.115e-6
    Jyy = 0.015962
    Jyz = 0.00003107
    Jzx = 0.00004547
    Jzy = 0.00003107
    Jzz = 0.00651814
    
    J = np.array([[Jxx, Jxy, Jxz], # satellite inertia matrix
                  [Jyx, Jyy, Jyz], 
                  [Jzx, Jzy, Jzz]])
    # J = np.array([[Jxx, 0, 0], # satellite inertia matrix
    #               [0, Jyy, 0], 
    #               [0, 0, Jzz]])
    # mass = 2.85087233 # satellite mass [kg]
    mass = 3.05353136
    
    sim_time = 0.1  # seconds  LOOK HERE: 90 degree rotation and 800 seconds shows really weird instability!!!
    dynamics_update_time = 0.01
    fsw_update_time = 0.1
    # viz_filename = f"{fsw_update_time:.2f}".replace('.', 'p') + "s_fsw_update_time"
    viz_filename = None
    
    sim_main(sim_time, J, mass, dynamics_update_time, fsw_update_time, viz_filename) # call and run simulation
    
    # J = np.array([[Jxx, -Jxy, -Jxz], # satellite inertia matrix with negative signs in front of off diagonal entries to account for OnShape convention (not sure yet if this is needed)
    #               [-Jyx, Jyy, -Jyz], 
    #               [-Jzx, -Jzy, Jzz]])
