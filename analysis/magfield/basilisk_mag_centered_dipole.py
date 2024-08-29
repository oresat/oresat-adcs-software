


"""
THIS IS A SCRIPT MODIFIED FROM scenarioMagneticFieldCenteredDipole.py from the Basilisk Package
"""


import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

bskPath = __path__[0]
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import magneticFieldCenteredDipole
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport)
from Basilisk.utilities import simSetPlanetEnvironment

#attempt to import vizard
from Basilisk.utilities import vizSupport
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, init_position, init_velocity, simulation_time_s, numDataPoints):
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
    simulationTimeStep = macros.sec2nano(10.)
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
    magModule2.g10 = -30926.00 / 1e9 * 0.5  # Tesla
    magModule2.g11 =  -2318.00 / 1e9 * 0.5  # Tesla
    magModule2.h11 =   5817.00 / 1e9 * 0.5  # Tesla
    magModule2.planetRadius = 6371.2 * 1000  # meters
    # REMOVED MAG FIELD LIMITS

    scSim.AddModelToTask(simTaskName, magModule2)



    # POSITION AND VELOCITY
    scObject.hub.r_CN_NInit = init_position  # m   - r_BN_N
    scObject.hub.v_CN_NInit = init_velocity  # m/s - v_BN_N


    # SIMULATION TIME
    simulationTime = macros.sec2nano(simulation_time_s)

    # Setup data logging before the simulation is initialized
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
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




    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs

    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    timeAxis = dataLog.times() * macros.NANO2SEC
    for idx in range(3):
        plt.plot(timeAxis, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [seconds]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    for idx in range(3):
        plt.plot(timeAxis, magData2[:, idx] * 1e9, '--',
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$B\_N_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [seconds]')
    plt.ylabel('Magnetic Field [nT]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)


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
        init_position,
        init_velocity,
        simulation_time_s = 1000,
        numDataPoints = 100
    )

