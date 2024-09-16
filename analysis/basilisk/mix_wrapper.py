
import inspect
import os

import numpy as np
from matplotlib import pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import simplePowerSink
from Basilisk.simulation import simpleBattery
from Basilisk.simulation import simpleSolarPanel
from Basilisk.simulation import eclipse
from Basilisk.simulation import spacecraft
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import unitTestSupport



from Basilisk import __path__
bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))




import basilisk_wrapper





def run(show_plots, step_time, stop_time, init_pos, init_vel, init_att, init_ang_vel, init_timestring):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        step_time: duration of a simulation step [seconds]
        stop_time: approximate duration of a simulation [seconds]

    """
    taskName = "simulation-task"               # arbitrary name (don't change)
    processName = "simulation-process"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    scenarioSim = SimulationBaseClass.SimBaseClass()
    simProc = scenarioSim.CreateNewProcess(processName)
    simProc.addTask(scenarioSim.CreateNewTask(taskName, macros.sec2nano(step_time)))

    # Create a spacecraft around Earth
    # initialize spacecraft object and set properties
    scObject, scObjectMsg, satLog = basilisk_wrapper.get_satellite('oresat', init_pos, init_vel, init_att, init_ang_vel)
    scenarioSim.AddModelToTask(taskName, scObject)
    scenarioSim.AddModelToTask(taskName, satLog)

    # Setup spice interface iwth earth and sun
    # store planet and sun msgs (which will be used later) and logs
    spiceObject, plMsg, plLog, sunMsg, sunLog= basilisk_wrapper.get_spice_earth_sun(scObject, init_timestring)
    scenarioSim.AddModelToTask(taskName, spiceObject, -1)
    scenarioSim.AddModelToTask(taskName, plLog)
    scenarioSim.AddModelToTask(taskName, sunLog)

    # Create magnetic field
    # Initialize magnetic field and mag logger
    magModule, magMsg, magLog = basilisk_wrapper.get_mag_model("magModel", scObject)
    scenarioSim.AddModelToTask(taskName, magModule)
    scenarioSim.AddModelToTask(taskName, magLog)

    # may need to attach epoch messages


    # Create an eclipse object so the panels don't always work
    eclipseObject, eclipseMsg, eclipseLog = basilisk_wrapper.get_eclipse_model("eclipseModel", scObjectMsg, sunMsg, plMsg)
    scenarioSim.AddModelToTask(taskName, eclipseObject)
    scenarioSim.AddModelToTask(taskName, eclipseLog)


    # Create a solar panel
    # Set the panel normal vector in the body frame, the area,
    solarPanel = basilisk_wrapper.get_solar_panel("solarPanel", scObjectMsg, eclipseMsg, sunMsg, [[1,0,0], 0.2*0.3, 0.20]) 
    spLog = solarPanel.nodePowerOutMsg.recorder()
    scenarioSim.AddModelToTask(taskName, solarPanel)
    scenarioSim.AddModelToTask(taskName, spLog)


    #   Create a simple power sink
    powerSink = basilisk_wrapper.get_power_sink("powerSink2", -3)
    psLog = powerSink.nodePowerOutMsg.recorder()
    scenarioSim.AddModelToTask(taskName, powerSink)
    scenarioSim.AddModelToTask(taskName, psLog)


    # Create a simpleBattery and attach the sources/sinks to it
    powerMonitor = basilisk_wrapper.get_power_monitor("powerMonitor", capacity=(10.0*3600.0), init_charge=(10.0*3600.0))
    powerMonitor.addPowerNodeToModel(solarPanel.nodePowerOutMsg)
    powerMonitor.addPowerNodeToModel(powerSink.nodePowerOutMsg)
    pmLog = powerMonitor.batPowerOutMsg.recorder()
    scenarioSim.AddModelToTask(taskName, powerMonitor)
    scenarioSim.AddModelToTask(taskName, pmLog)




    # SIMULATION
    # Need to call the self-init and cross-init methods
    # Start the simulation
    scenarioSim.InitializeSimulation()
    scenarioSim.ConfigureStopTime(macros.sec2nano(stop_time))        # seconds to stop simulation
    scenarioSim.ExecuteSimulation()



    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)

    # Position and Time
    posData = satLog.r_BN_N
    timeAxis = satLog.times() * macros.NANO2HOUR
    #plLog
    #sunLog

    # Magnetic Field
    magData = magLog.magField_N

    # Solar
    eclipseData = eclipseLog.shadowFactor
    supplyData = spLog.netPower
    sinkData = psLog.netPower
    storageData = pmLog.storageLevel
    netData = pmLog.currentNetPower


    time_ns = satLog.times()

    #   Plot the power states
    figureList = {}



    if show_plots:
        plt.show()
    plt.close("all")

    return figureList




#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    
    timeInitString = '2025 MAY 04 07:47:48.965 (UTC)'
    init_position = [-4963946.392216118, 4601467.815050239, -1311445.5818653065]
    init_velocity = [1731.502687329283, -238.55435888532116, -7398.92444558897] 
    init_MRP_attitude = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    init_ang_velocity = [[0.001], [-0.001], [0.001]]

    run(
        False,  # show_plots
        step_time = 10.0,
        stop_time = 1000.0,
        init_pos = init_position,
        init_vel = init_velocity,
        init_att = init_MRP_attitude,
        init_ang_vel = init_ang_velocity,
        init_timestring = timeInitString
    )
