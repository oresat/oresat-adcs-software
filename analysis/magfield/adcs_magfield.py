import numpy as np
from sgp4.api import Satrec

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__

bskPath = __path__[0]
from Basilisk.simulation import spacecraft
from Basilisk.simulation import magneticFieldCenteredDipole
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport)
from Basilisk.utilities import simSetPlanetEnvironment

from Basilisk.utilities import vizSupport
fileName = os.path.basename(os.path.splitext(__file__)[0])



def basilisk_run_dipole(init_position, init_velocity, sim_time, sim_time_step, num_data_points):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        simulation_time_s (int): Simulation time in seconds
        numDataPoints(int): number of desired data points
    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass() # create a sim module as an empty container
    dynProcess = scSim.CreateNewProcess(simProcessName) # create the simulation process

    ### SET SIMULATION TIME
    simulationTimeStep = macros.sec2nano(sim_time_step)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # SETUP THE SIMULATION TASKS/OBJECTS
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    scSim.AddModelToTask(simTaskName, scObject) # add spacecraft object to the simulation process

    # GRAVITY MODEL
    gravFactory = simIncludeGravBody.gravBodyFactory() # set up gravity body
    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body

    req = planet.radEquator

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    # create the magnetic field
    magModule = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()  # default is Earth centered dipole module
    magModule.ModelTag = "CenteredDipole"
    magModule.addSpacecraftToModel(scObject.scStateOutMsg)  # this command can be repeated if multiple

    simSetPlanetEnvironment.centeredDipoleMagField(magModule, 'earth')
    scSim.AddModelToTask(simTaskName, magModule)

    # ELLIPTIC MAG FIELD
    magModule2 = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
    magModule2.ModelTag = "CenteredDipole2"
    magModule2.addSpacecraftToModel(scObject.scStateOutMsg)

    # set the 2nd magnetic field through custom dipole settings
    magModule2.g10 = -29404.8 / 1e9  # Tesla
    magModule2.g11 =  -1450.9 / 1e9  # Tesla
    magModule2.h11 =   4652.5 / 1e9  # Tesla
    magModule2.planetRadius = 6371.2 * 1000  # meters
    # REMOVED MAG FIELD LIMITS

    scSim.AddModelToTask(simTaskName, magModule2)


    # POSITION AND VELOCITY
    scObject.hub.r_CN_NInit = init_position  # m   - r_BN_N
    scObject.hub.v_CN_NInit = init_velocity  # m/s - v_BN_N


    # SIMULATION TIME
    simulationTime = macros.sec2nano(sim_time)

    # Setup data logging before the simulation is initialized
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, num_data_points)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)
    mag2Log = magModule2.envOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, mag2Log)




    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    magData2 = mag2Log.magField_N

    np.set_printoptions(precision=16)


    return posData, magData2




import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import magneticFieldWMM
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport)

#attempt to import vizard
from Basilisk.utilities import vizSupport


def basilisk_run_WMM(init_position, init_velocity, sim_time, sim_time_step, num_data_points, init_epoch):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        orbitCase (str): {'circular', 'elliptical'}

    """


    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    
    # SET SIMULATION TIME
    simulationTimeStep = macros.sec2nano(sim_time_step)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    
    req = planet.radEquator

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    # create the magnetic field
    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.ModelTag = "WMM"
    magModule.dataPath = bskPath + '/supportData/MagneticField/'


    # set epoch date/time message
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(init_epoch)

    # add spacecraft to the magnetic field module so it can read the sc position messages
    magModule.addSpacecraftToModel(scObject.scStateOutMsg)  # this command can be repeated if multiple

    # add the magnetic field module to the simulation task stack
    scSim.AddModelToTask(simTaskName, magModule)

  
    #
    #   initialize Spacecraft States with the initialization variables
    #

    ### DEFINE P rN vN
    scObject.hub.r_CN_NInit = init_position  # m   - r_BN_N
    scObject.hub.v_CN_NInit = init_velocity  # m/s - v_BN_N

 
    simulationTime = macros.sec2nano(sim_time)

    # connect messages
    magModule.epochInMsg.subscribeTo(epochMsg)

    #   Setup data logging before the simulation is initialized
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, num_data_points)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    magLog = magModule.envOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, magLog)

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                                  # saveFile=fileName,
                                                  )
        viz.epochInMsg.subscribeTo(epochMsg)

        viz.settings.show24hrClock = 1
        viz.settings.showDataRateDisplay = 1

    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    magData = magLog.magField_N
    posData = dataLog.r_BN_N

    np.set_printoptions(precision=16)
    

    return posData, magData







from oresat_adcs.classes import dynamics, jday
from oresat_adcs.configuration import environment

def dummy_array(x_0, v_0):
    return np.array([x_0, v_0, np.array([1, 0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0, 0])], dtype=object)

if __name__ == "__main__":
    tle1 = "1 98867U          24238.20000000  .00000000  00000-0  20199-3 0    06"
    tle2 = "2 98867  97.4404 314.0487 0008202 330.6740  42.2938 15.18964090    01"
    sat_sgp4 = Satrec.twoline2rv(tle1, tle2)

    my_env = environment.OrbitalEnvironment(hi_fi=True)

    # units in meters and meters per second
    x_0 =[5459160.934523222, -4052774.5387697653, 1098818.5050856567]
    v_0 = [-1563.678108182039, -81.69307509455707, 7447.743282587973]
    t_0 = (2024, 9, 4, 12, 0, 0)

    my_jclock = jday.JClock(*t_0)
    julian_float = my_jclock.julian_date()
    e, r, v = sat_sgp4.sgp4(julian_float, 0.0)
    x_0 = [thing*1000 for thing in r]
    v_0 = [thing*1000 for thing in v]

    adcs_mag = []
    adcs_pos = []
    for _ in range(1001):

        julian_float = my_jclock.julian_date()
        e, r, v = sat_sgp4.sgp4(julian_float, 0.0)
        r = [thing*1000 for thing in r]
        v = [thing*1000 for thing in v]

        adcs_pos.append(r)
        my_array = dummy_array(r, v)
        my_state = dynamics.SatelliteState(my_array, my_jclock) 
        my_state.update()

        thing = my_env.magnetic_field(my_state)
        adcs_mag.append(thing)

        my_jclock.tick(1)

    adcs_pos = np.array(adcs_pos)
    adcs_mag = np.array(adcs_mag)


    pos_dipole, mag_dipole = basilisk_run_dipole(x_0, v_0, sim_time=1000., sim_time_step=1., num_data_points=1000)
    data_dipole = mag_dipole.T


    pos_WMM, mag_WMM = basilisk_run_WMM(x_0, v_0, sim_time=1000., sim_time_step=1., num_data_points = 1000, init_epoch = '2024 September 4, 12:0:0.0 (UTC)')

    adcs_WWM_mag_error = adcs_mag - mag_WMM

    data_WMM = mag_WMM.T
    # Orbit, I think
    #fig = plt.figure()
    #ax = plt.axes(projection='3d')
    #ax.plot3D(0, 0, 0, marker='o', label='reference')
    #ax.plot3D(*adcs_pos.T, marker='+', label='ADCS Model')
    #ax.plot3D(*pos_WMM.T, marker='x', label='WMM')
    #ax.legend()

    # Magnetic field
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(0, 0, 0, marker='o', label='reference')
    ax.plot3D(*adcs_mag.T, marker='.', label='ADCS Model')
    ax.plot3D(*data_WMM, marker='.', label='WMM')
    ax.plot3D(*data_dipole, marker='x', label='IGRF, 2020')
    ax.legend()
    plt.show()
