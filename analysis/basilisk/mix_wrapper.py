
import inspect
import os
import json

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
from Basilisk.simulation import coarseSunSensor

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
    #solarPanel = basilisk_wrapper.get_solar_panel("solarPanel", scObjectMsg, eclipseMsg, sunMsg, [[1,0,0], 1, 1]) 
    #spLog = solarPanel.nodePowerOutMsg.recorder()
    #scenarioSim.AddModelToTask(taskName, solarPanel)
    #scenarioSim.AddModelToTask(taskName, spLog)

    # create multiple solar panels
    directions = {"x+": [1, 0, 0],
                  "x-": [-1, 0, 0],
                  "y+": [0, 1, 0],
                  "y-": [0, -1, 0],
                  "z+": [0, 0, 1],
                  "z-": [0, 0, -1]}


    # create solar panels
    solar_panels = dict()
    panel_logs = dict()
    for direction, vector in directions.items():
        solar_panels[direction] = basilisk_wrapper.get_solar_panel("solarPanel_"+direction, scObjectMsg, eclipseMsg, sunMsg, [vector, 1, 1]) 
        panel_logs[direction] = solar_panels[direction].nodePowerOutMsg.recorder()
        scenarioSim.AddModelToTask(taskName, solar_panels[direction])
        scenarioSim.AddModelToTask(taskName, panel_logs[direction])
        

    # coarse solar sensor
    sun_sensors = dict()
    sun_logs = dict()
    for direction,vector in directions.items():
        # later, see if we can do a constallation
        sun_sensors[direction] = coarseSunSensor.CoarseSunSensor()
        sun_sensors[direction].ModelTag = "sunSensor_"+direction
        # field of view from normal is 90 degrees
        sun_sensors[direction].fov = 90. * macros.D2R
        sun_sensors[direction].nHat_B = np.array(vector)
        sun_sensors[direction].sunInMsg.subscribeTo(sunMsg)
        sun_sensors[direction].stateInMsg.subscribeTo(scObjectMsg)
        sun_sensors[direction].sunEclipseInMsg.subscribeTo(eclipseMsg)
        sun_logs[direction] = sun_sensors[direction].cssDataOutMsg.recorder()
        scenarioSim.AddModelToTask(taskName, sun_sensors[direction])
        scenarioSim.AddModelToTask(taskName, sun_logs[direction])

    #CSS_test = coarseSunSensor.CoarseSunSensor()
    #CSS_test.ModelTag = "CSS_Test_sensor"

    # set up sun vector
    #CSS_test.fov = 90. * macros.D2R
    #CSS_test.nHat_B = np.array([1, 0, 0])
    #CSS_test.sunInMsg.subscribeTo(sunMsg)
    #CSS_test.stateInMsg.subscribeTo(scObjectMsg)
    #scenarioSim.AddModelToTask(taskName, CSS_test)
    #CSS_test_log = CSS_test.cssDataOutMsg.recorder()
    #scenarioSim.AddModelToTask(taskName, CSS_test_log)

    #   Create a simple power sink
    #powerSink = basilisk_wrapper.get_power_sink("powerSink2", -3)
    #psLog = powerSink.nodePowerOutMsg.recorder()
    #scenarioSim.AddModelToTask(taskName, powerSink)
    #scenarioSim.AddModelToTask(taskName, psLog)


    # Create a simpleBattery and attach the sources/sinks to it
    #powerMonitor = basilisk_wrapper.get_power_monitor("powerMonitor", capacity=(10.0*3600.0), init_charge=(10.0*3600.0))
    #powerMonitor.addPowerNodeToModel(solarPanel.nodePowerOutMsg)
    #powerMonitor.addPowerNodeToModel(powerSink.nodePowerOutMsg)
    #pmLog = powerMonitor.batPowerOutMsg.recorder()
    #scenarioSim.AddModelToTask(taskName, powerMonitor)
    #scenarioSim.AddModelToTask(taskName, pmLog)




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
   # print(dir(sunLog))
   # print(dir(eclipseLog))
    eclipseData = eclipseLog.shadowFactor
    #supplyData = spLog.netPower
    panelData = { direction:[float(power) for power in powerLog.netPower] for direction,powerLog in panel_logs.items() }
    #sinkData = psLog.netPower
    #storageData = pmLog.storageLevel
    #netData = pmLog.currentNetPower

    #print(CSS_test_log.OutputData)
    #sunData = CSS_test_log.OutputData
    sunData = { direction:[float(reading) for reading in sunLog.OutputData] for direction,sunLog in sun_logs.items() }


    panel_data = list()
    for ii in range(len(timeAxis)):
        panel_data.append([direction[ii] for direction in panelData.values()])
    with open('panel.txt', 'w') as fd:
        for line in panel_data:
            fd.write(str(line) + "\n")

    sun_data = [["sun_exposure", "x+", "x-", "y+", "y-", "z+", "z-"]]
    for ii in range(len(timeAxis)):
        sun_data.append([float(eclipseData[ii])] + [direction[ii] for direction in sunData.values()])
    with open('sun.csv', 'w') as fd:
        for line in sun_data:
            fd.write(",".join([str(thing) for thing in line]) + "\n")



    temp_data = [["time", "mag_x", "mag_y", "mag_z", "is_eclipsed"]]
    for ii in range(len(timeAxis)):
        temp_data.append([float(timeAxis[ii])] + [float(magd) for magd in magData[ii]] + [float(eclipseData[ii])])
    with open('output.txt','w') as fd:
        for line in temp_data:
            fd.write(str(line) + "\n")



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
    
    timeInitString = '2024 DEC 21 23:57:00.000 (UTC)'
    init_position = [2204478.478478075, 6521207.613663136, -199008.2069751579]
    init_velocity = [1002.5835456934169, -110.58217266418544, 7542.886276905129]
    init_MRP_attitude = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    init_ang_velocity = [[0.01], [-0.01], [0.01]]

    run(
        False,  # show_plots
        step_time = 10.0,
        stop_time = 24.0*60.0*60.0,
        init_pos = init_position,
        init_vel = init_velocity,
        init_att = init_MRP_attitude,
        init_ang_vel = init_ang_velocity,
        init_timestring = timeInitString
    )
