
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

from Basilisk import __path__
bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))



import basilisk_wrapper
# add model tags



def run(show_plots, init_pos, init_vel, init_att, init_ang_vel):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """
    taskName = "unitTask"               # arbitrary name (don't change)
    processname = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    scenarioSim = SimulationBaseClass.SimBaseClass()
    testProc = scenarioSim.CreateNewProcess(processname)
    testProc.addTask(scenarioSim.CreateNewTask(taskName, macros.sec2nano(1.0)))

    # Create a spacecraft around Earth
    # initialize spacecraft object and set properties
    scObject, scObjectMsg, dataLog = basilisk_wrapper.get_satellite('oresat', init_pos, init_vel, init_att, init_ang_vel)
    scenarioSim.AddModelToTask(taskName, scObject)
    scenarioSim.AddModelToTask(taskName, dataLog)


    # clear prior gravitational body and SPICE setup definitions
    # setup Spice interface for some solar system bodies
    timeInitString = '2021 MAY 04 07:47:48.965 (UTC)'
    spiceObject, plMsg, plLog, sunMsg, sunLog = basilisk_wrapper.get_spice_earth_sun(scObject, timeInitString)
    scenarioSim.AddModelToTask(taskName, spiceObject, -1)
    scenarioSim.AddModelToTask(taskName, plLog)
    scenarioSim.AddModelToTask(taskName, sunLog)



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
    scenarioSim.ConfigureStopTime(macros.sec2nano(10000.))        # seconds to stop simulation
    scenarioSim.ExecuteSimulation()



    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    posData = dataLog.r_BN_N
    eclipseData = eclipseLog.shadowFactor

    print(len(posData))

    supplyData = spLog.netPower
    sinkData = psLog.netPower
    storageData = pmLog.storageLevel
    netData = pmLog.currentNetPower

    print(len(netData))

    tvec = spLog.times()
    tvec = tvec * macros.NANO2HOUR

    #   Plot the power states
    figureList = {}
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    plt.plot(tvec, storageData/3600., label='Stored Power (W-Hr)')
    plt.plot(tvec, netData, label='Net Power (W)')
    plt.plot(tvec, supplyData, label='Panel Power (W)')
    plt.plot(tvec, sinkData, label='Power Draw (W)')
    plt.xlabel('Time (Hr)')
    plt.ylabel('Power (W)')
    plt.grid(True)
    plt.legend()

    pltName = "scenario_powerDemo"
    figureList[pltName] = plt.figure(1)


    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    for idx in range(3):
        plt.plot(posData[: ,idx])

    pltName = "position"
    figureList[pltName] = plt.figure(2)


    if show_plots:
        plt.show()
    plt.close("all")

    return figureList


#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    init_position = [-4963946.392216118, 4601467.815050239, -1311445.5818653065]
    init_velocity = [1731.502687329283, -238.55435888532116, -7398.92444558897] 
    init_MRP_attitude = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    init_ang_velocity = [[0.001], [-0.001], [0.001]]

    run(
        True,  # show_plots
        init_pos = init_position,
        init_vel = init_velocity,
        init_att = init_MRP_attitude,
        init_ang_vel = init_ang_velocity
    )
