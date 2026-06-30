

import inspect
import os
import argparse

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
from Basilisk.simulation import magneticFieldWMM
from Basilisk.simulation import starTracker
from Basilisk.simulation import imuSensor
from Basilisk.simulation import magnetometer


from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

from Basilisk import __path__
bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))



# anything attribute that has "OutMsgs" will have a .recorder() function to generate a log
# see message payloads for attributes to get data from




def get_satellite(modelTag, rI, init_pos=None, init_vel=None, init_att=None, init_ang_vel=None):
    '''
        rI = Rotational Inertia of Sat
        init_pos = initial position in ECI
        init_vel = initial velocity in ECI
        init_att = initial attitude in MRP
        init_ang_vel = initial angular velocities
    returns
        scObject (satellite)
    '''
    satellite = spacecraft.Spacecraft()
    satellite.ModelTag = modelTag
   # I = [16.50e7, 71145.23, 457069.94,
    #     71145.23, 15.96e7, 310717.76,
     #    457069.94, 310717.76, 65.18e6]
    satellite.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(rI)
    if init_pos is not None:
        satellite.hub.r_CN_NInit = init_pos
    if init_vel is not None:
        satellite.hub.v_CN_NInit = init_vel
    if init_att is not None:
        satellite.hub.sigma_BNInit = init_att
    if init_ang_vel is not None:
        satellite.hub.omega_BN_BInit = init_ang_vel

    satMsg = satellite.scStateOutMsg
    satLog = satMsg.recorder()
    return satellite, satMsg, satLog



def get_gravity_factory_earth(scObject):
    ''' Adds gravity parameters to scenario object

    args:
        scObject: scenario object (satellite)

    returns:
        gravityFactory
    '''
    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)
    return gravFactory


def get_spice_earth_sun(scObject, timeInitString, zeroBase='Earth'):

    '''
        timeInitString:
        zeroBase: Where to place the coordinate system, earth by default
    '''
   # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    sun = gravFactory.createSun()
    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    # setup Spice interface for some solar system bodies
    spiceObject = gravFactory.createSpiceInterface(time=timeInitString)
    spiceObject.zeroBase = zeroBase

    plMsg = spiceObject.planetStateOutMsgs[0]
    plLog = plMsg.recorder()

    sunMsg = spiceObject.planetStateOutMsgs[1]
    sunLog = sunMsg.recorder()

    return spiceObject, plMsg, plLog, sunMsg, sunLog


def get_mag_model(modelTag, scObject):
    '''
    args:
        scObject: scenario object (satellite)

    returns
        magnetic field model
        magnetic field message object
        magnetic field recorder (log)
    '''
    # create the magnetic field
    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.configureWMMFile(str(get_path(DataFile.MagneticFieldData.WMM)))
    magModule.ModelTag = modelTag

    magModule.addSpacecraftToModel(scObject.scStateOutMsg)  # this command can be repeated if multiple
    magMsg = magModule.envOutMsgs[0]
    magLog = magMsg.recorder()
    return magModule, magMsg, magLog




def get_eclipse_model(modelTag, scObjectMsg, sunMsg, planetMsg):
    '''
        scObjectMsg: satellite messages
        sourceObjectMsg: sun messages
        sinkObjectMsg: earth (or planet) messages
    '''
    eclipseObject = eclipse.Eclipse()
    eclipseObject.ModelTag = modelTag
    eclipseObject.addSpacecraftToModel(scObjectMsg)
    eclipseObject.addPlanetToModel(planetMsg)
    eclipseObject.sunInMsg.subscribeTo(sunMsg)

    eclipseMsg = eclipseObject.eclipseOutMsgs[0]
    eclipseLog = eclipseMsg.recorder()
    return eclipseObject, eclipseMsg, eclipseLog






def get_power_sink(modelTag, nodePowerOut):
    '''
        modelTag: name for model
        nodePowerOut: power output in watts. Negative means it consumes power
    '''
    powerSink = simplePowerSink.SimplePowerSink()
    powerSink.ModelTag = modelTag
    powerSink.nodePowerOut = -3.  # Watts

    return powerSink




def get_solar_panel(modelTag, scObjectMsg, eclipseMsg, sunMsg, parameters):
    '''
        modelTag: name for model
        scObjectMsg: satellite messages
        eclipseMsg: eclipse messages
        sunMsg: sun messages
        parameters: additional parameters [normal vector body ref frame, area, something]

    returns
        solarPanel: simpleSolarPanel()
        solarPanelMsg: nodePowerOutMsg
        solarPanelLog: nodePowerOutMsg.recorder()
    '''
    solarPanel = simpleSolarPanel.SimpleSolarPanel()
    solarPanel.ModelTag = modelTag
    solarPanel.stateInMsg.subscribeTo(scObjectMsg)
    solarPanel.sunEclipseInMsg.subscribeTo(eclipseMsg)
    solarPanel.sunInMsg.subscribeTo(sunMsg)
    solarPanel.setPanelParameters(*parameters)

    return solarPanel


def get_power_monitor(modelTag, capacity, init_charge):
    '''
        capacity: storage capacity in joules
        init_charge: initial charge in joules
        power_nodes: list of power nodes to add. must have the nodePowerOutMsg attribute
    '''
    powerMonitor = simpleBattery.SimpleBattery()
    powerMonitor.ModelTag = modelTag
    powerMonitor.storageCapacity = capacity
    powerMonitor.storedCharge_Init = init_charge
    return powerMonitor


def make_star_tracker(modelTag, scObjMsg, record_t, dcm_CB=None, noise_std=None, w_bounds=None):
    '''
        modelTag: name of model
        dcm_CB (optional): orientation relative to body (body to case)
        noiseStd (optional): noise standard deviation (assumed to be the same for all axes)
        bounds (optional): senor noise/walk bounds (may default to +- 5 standard errors)
    '''

    starTrackerSensor = starTracker.StarTracker()
    starTrackerSensor.ModelTag = modelTag
    
    if dcm_CB is not None:
        # if there is a body-to-case rotation, apply it
        starTrackerSensor.dcm_CB = dcm_CB

    if noise_std is not None:
        # if there is noise
        starTrackerSensor.PMatrix = np.eye(3) * (noise_std**2)

    if w_bounds is not None:
        starTrackerSensor.setWalkBounds([w_bounds, w_bounds, w_bounds])

    starTrackerSensor.scStateInMsg.subscribeTo(scObjMsg)
    starTrackerRec = starTrackerSensor.sensorOutMsg.recorder(record_t)

    # starTrackerSensor.UpdateState(0)
    return starTrackerSensor, starTrackerRec


def make_gyro(modelTag, scObjMsg, record_t, dcm=None, noise_std=None, e_bounds=None, w_bounds=None):
    '''
        makes a gyro-only imu sensor, this could be expanded in the future

        modelTag: name of model
        scObjMsg: the messager of the scObject
        record_t: time delta for the recorder
        dcm (optional): orientation relative to body (body to platform)
        noise_std (optional): noise standard deviation (assumed to be the same for all axes)
        e_bounds (optional): sensor error bounds (may default to +- 3 standard errors)
        w_bounds (optional): sensor walk bounds (may default to +- 5 standard errors)
    '''
    gyroSensor = imuSensor.ImuSensor()
    gyroSensor.ModelTag = modelTag
    
    if dcm is not None:
        # if there is a body-to-case rotation, apply it
        # syntax is different and this is untested
        gyroSensor.dcm_PB = dcm

    if noise_std is not None:
        # if there is noise, apply it
        gyroSensor.PMatrixGyro = np.eye(3) * (noise_std**2)

    if e_bounds is not None:
        # if there are error bounds, apply them
        gyroSensor.setErrorBoundsGyro([e_bounds, e_bounds, e_bounds])

    if w_bounds is not None:
        # if there are walk bounds, apply them
        gyroSensor.setWalkBoundsGyro([w_bounds, w_bounds, w_bounds])

    # set up message and recorder pipelines
    gyroSensor.scStateInMsg.subscribeTo(scObjMsg)
    gyroRec = gyroSensor.sensorOutMsg.recorder(record_t)

    # assumes the scObject state has already been set
    gyroSensor.UpdateState(0)
    return gyroSensor, gyroRec



def make_magnetometer(modelTag, magMsg, scObjMsg, record_t, dcm=None, noise_std=None, w_bounds=None):
    '''
        makes a magnetometer, this could be expanded in the future

        modelTag: name of model
        magMsg: message from the magnetic field 
        scObjMsg: the messager of the scObject
        record_t: time delta for the recorder
        dcm (optional): orientation relative to body (body to sensor frame)
        noiseStd (optional): noise standard deviation (assumed to be the same for all axes)
        e_bounds (optional): sensor error bounds (may default to +- 3 standard errors)
        w_bounds (optional): sensor walk bounds (may default to +- 5 standard errors)
    '''
    magSensor = magnetometer.Magnetometer()
    magSensor.ModelTag = modelTag
    
    if dcm is not None:
        # if there is a body-to-case rotation, apply it
        # syntax is different and this is untested
        magSensor.dcm_SB = dcm


    if noise_std is not None:
        # if there is noise, apply it
        magSensor.senNoiseStd = [noise_std, noise_std, noise_std]

    if w_bounds is not None:
        # if there are walk bounds, apply them
        magSensor.walkBounds = [w_bounds, w_bounds, w_bounds]

    # set up message and recorder pipelines
    magSensor.magInMsg.subscribeTo(magMsg)
    magSensor.stateInMsg.subscribeTo(scObjMsg)
    magRec = magSensor.tamDataOutMsg.recorder(record_t)

    # assumes the scObject state has already been set
    magSensor.UpdateState(0)
    return magSensor, magRec

