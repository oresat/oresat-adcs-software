

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

from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import ReactionWheelPower

from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

from Basilisk.architecture import messaging

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


def make_spice_earth_sun_moon(sc_object, init_time_string):

    # start with gravity factory
    grav_factory = simIncludeGravBody.gravBodyFactory()

    # create bodies

    # Earth is the central body
    earth = grav_factory.createEarth()
    earth.isCentralBody = True
    # Create the sun
    sun = grav_factory.createSun()
    # Create the moon
    moon = grav_factory.createMoon()

    grav_factory.addBodiesTo(sc_object)

    # Now create the spice object
    # eventually add better ephemeris data
    spice_object = grav_factory.createSpiceInterface(
        bskPath + "/supportData/EphemerisData",
        time = init_time_string,
        epochInMsg = True,
    )
    spice_object.zeroBase = "Earth"

    grav_factory.addBodiesTo(sc_object)

    earth.planetBodyInMsg.subscribeTo(spice_object.planetStateOutMsgs[0])
    sun.planetBodyInMsg.subscribeTo(spice_object.planetStateOutMsgs[1])
    moon.planetBodyInMsg.subscribeTo(spice_object.planetStateOutMsgs[2])

    return grav_factory, spice_object, earth, sun, moon



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




def get_eclipse_model(
    model_tag, 
    sc_object_msg, 
    sun_msg, 
    planet_msg):
    '''
        scObjectMsg: satellite messages
        sourceObjectMsg: sun messages
        sinkObjectMsg: earth (or planet) messages
    '''
    eclipse_object = eclipse.Eclipse()
    eclipse_object.ModelTag = model_tag
    eclipse_object.addSpacecraftToModel(sc_object_msg)
    eclipse_object.sunInMsg.subscribeTo(sun_msg)
    eclipse_object.addPlanetToModel(planet_msg)

    eclipse_msg = eclipse_object.eclipseOutMsgs[0]
    eclipse_rec = eclipse_msg.recorder()
    return eclipse_object, eclipse_msg, eclipse_rec


def make_sun_sensor(
    model_tag,
    sc_object_msg,
    sun_msg,
    eclipse_msg,
    nHat_B,
    fov = 90 * macros.D2R
):
    '''Make a sun sensor

    Parameters
    ----------
    model_tag
        name of model
    sc_object_msg
        Spacecraft message object
    sun_msg
        Sun planet body message
    eclipse_msg
        Eclipse model message
    nHat_B
        Direction of the sun sensor
    fov
        Field of view angle, in radians
    '''
    pass

def get_eclipse_multiple_model(
    model_tag, 
    sc_object_msg, 
    sun_msg, 
    planet_msgs):
    '''Allows for eclipsing of multiple bodies
    Parameters
    ----------
    model_tag: 
        model tag name
    sc_object_msg:
        Spacecraft object message
    sun_msg
        Solar source object message
    planet_msgs
        List of solar-blocking bodies
    '''
    eclipse_object = eclipse.Eclipse()
    eclipse_object.ModelTag = model_tag
    eclipse_object.addSpacecraftToModel(sc_object_msg)
    eclipse_object.sunInMsg.subscribeTo(sun_msg)
    for p_msg in planet_msgs:
        eclipse_object.addPlanetToModel(p_msg)

    eclipse_msg = eclipse_object.eclipseOutMsgs[0]
    eclipse_rec = eclipse_msg.recorder()
    return eclipse_object, eclipse_msg, eclipse_rec






def make_power_sink(model_tag, node_power_out):
    '''
    Parameters
    ----------
    model_tag
        name for model
    node_power_out: 
        power output in watts,  negative means it consumes power
    '''
    power_sink = simplePowerSink.SimplePowerSink()
    power_sink.ModelTag = model_tag
    power_sink.nodePowerOut = node_power_out  # Watts
    ps_msg = power_sink.nodePowerOutMsg
    ps_rec = ps_msg.recorder()
    return power_sink, ps_msg, ps_rec




def make_solar_panel(
    model_tag,
    sc_object_msg,
    sun_msg,
    eclipse_msg,
    parameters
):
    '''
    Parameters
    ----------
    model_tag: name for model
    sc_object_msg: satellite messages
    sun_msg: sun messages
    eclipse_msg: eclipse messages
    parameters: additional parameters [normal vector body ref frame, area, something]

    returns
        solarPanel: simpleSolarPanel()
        solarPanelMsg: nodePowerOutMsg
        solarPanelLog: nodePowerOutMsg.recorder()
    '''
    solar_panel = simpleSolarPanel.SimpleSolarPanel()
    solar_panel.ModelTag = model_tag
    solar_panel.stateInMsg.subscribeTo(sc_object_msg)
    solar_panel.sunInMsg.subscribeTo(sun_msg)
    solar_panel.sunEclipseInMsg.subscribeTo(eclipse_msg)
    solar_panel.setPanelParameters(*parameters)

    sp_msg = solar_panel.nodePowerOutMsg
    sp_rec = sp_msg.recorder()
    return solar_panel, sp_msg, sp_rec


def make_power_monitor(
    model_tag: str, 
    storage_capacity: float | int, 
    init_charge: float | int
):
    '''Create a power montior aka battery
    
    Parameters
    ----------
    model_tag:
        model tag
    storage_capacity: 
        total capacity in joules (W*s)
    init_charge: 
        initial charge in joules (W*s)
    '''
    battery_model = simpleBattery.SimpleBattery()
    battery_model.ModelTag = model_tag
    battery_model.storageCapacity = storage_capacity
    battery_model.storedCharge_Init = init_charge

    battery_msg = battery_model.batPowerOutMsg
    battery_rec = battery_msg.recorder()
    return battery_model, battery_msg, battery_rec



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


def make_rw_set(
    model_tag: str, 
    sc_object,
    rw_G: np.ndarray, 
    rot_inertia: float,
    use_max_torque: bool, 
    max_speed: float, 
    max_torque: float,
    has_jitter: bool = False,
):
    """
    Parameters
    model_tag
        model name for reaction wheel set
    sc_object
        sc object to attach to
    rw_G
        numpy matrix with reaction wheel axes
    rot_inertia
        rw rotational inertia about its spin axis
    use_max_torque
        satureate the reaction wheel torque
    max_speed
        maximum speed of reaction wheel
    max_torque
        maximum torque of reaction wheel
    base_power
        base power consumption when reaction wheel is on
    eff_elec_to_mech
        efficency converting electrical energy to mechanical energy, defaults to 1?
    eff_mech_to_elec
        energy recovery from braking 
        -1.0 means all power to brake
        0 no power required to brake (coasting)
        1 complete regenerative braking
    has_jitter
        declares if all the wheels have jitter or are balanced
    """
    varRWModel = messaging.JitterSimple if has_jitter else messaging.BalancedWheels

    RWFactory = simIncludeRW.rwFactory()

    for i in range(len(rw_G[0])): 
        # create number of reaction wheels equal to wheels defined in G matrix
        axis = rw_G[:,i]
        RWFactory.create(
            rwType = "custom",              # unique name
            gsHat_B = axis,                  # spin axis
            Js = rot_inertia,       # wheel inertia
            useMaxTorque = use_max_torque,    # disable max torque check
            Omega_max = max_speed,    # max speed
            u_max = max_torque,
            RWModel=varRWModel
        )

    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = model_tag


    return RWFactory, rwStateEffector

def make_rw_powers(
    model_tag: str,
    num_wheels: int,
    base_power: float = 0.0,
    eff_elec_to_mech: float = 1.0,
    eff_mech_to_elec: float = -1.0,
):

    rw_power_list = []
    rw_power_msg_list = []
    rw_power_rec_list = []
    for ii in range(num_wheels):
        rw_power = ReactionWheelPower.ReactionWheelPower()
        rw_power.ModelTag = model_tag + "_" + str(ii)
        rw_power.basePowerNeed = base_power
        rw_power.elecToMechEfficiency = eff_elec_to_mech
        rw_power.mechToElecEfficiency = eff_mech_to_elec

        rw_power_list.append(rw_power)
        rw_power_msg_list.append(rw_power.nodePowerOutMsg)
        rw_power_rec_list.append(rw_power.nodePowerOutMsg.recorder())

    return rw_power_list, rw_power_msg_list, rw_power_rec_list



