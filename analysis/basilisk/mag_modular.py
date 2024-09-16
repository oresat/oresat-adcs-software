

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


import basilisk_wrapper




def run(show_plots, step_time, stop_time, init_pos, init_vel):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        orbitCase (str): {'circular', 'elliptical'}

    """


    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create the simulation process
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, macros.sec2nano(step_time)))


    # Initialize satellite and data logger
    scObject, scObjectMsg, dataLog = basilisk_wrapper.get_satellite("oresat", init_pos, init_vel)
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, dataLog)


    # Add gravity fields to satellite
    gravFactory = basilisk_wrapper.get_gravity_factory_earth(scObject)


    # Initialize magnetic field and mag logger
    magModule, magMsg, magLog = basilisk_wrapper.get_mag_model('magModel', scObject)
    scSim.AddModelToTask(simTaskName, magModule)
    scSim.AddModelToTask(simTaskName, magLog)




    # configure a simulation stop time and execute the simulation run
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(stop_time))
    scSim.ExecuteSimulation()

    # retrieve the logged data
    magData = magLog.magField_N
    posData = dataLog.r_BN_N

    np.set_printoptions(precision=16)



    # plot the results
    
    plt.close("all")  # clears out plots from earlier test runs

    timeAxis = dataLog.times() * macros.NANO2HOUR
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    rData = []
    for idx in range(0, len(posData)):
        rMag = np.linalg.norm(posData[idx])
        rData.append(rMag / 1000.)
    plt.plot(timeAxis, rData, color='#aa0000')

    plt.xlabel('Time [orbits]')
    plt.ylabel('Radius [km]')
    plt.ylim(min(rData)*0.9, max(rData)*1.1)
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    for idx in range(3):
        plt.plot(timeAxis, magData[:, idx] *1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$B\_N_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel('Magnetic Field [nT]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)
    

    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    for idx in range(3):
        plt.plot(timeAxis, posData[:, idx] *1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'Position')
    plt.legend(loc='lower right')
    plt.xlabel('Time [hours]')
    plt.ylabel('Position [m]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(3)



    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList



#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    init_position = [-4963946.392216118, 4601467.815050239, -1311445.5818653065]
    init_velocity = [1731.502687329283, -238.55435888532116, -7398.92444558897] 


    run(
        True,          # show_plots
        step_time = 10.0, 
        stop_time = 1000.0,
        init_pos = init_position,
        init_vel = init_velocity
    )
