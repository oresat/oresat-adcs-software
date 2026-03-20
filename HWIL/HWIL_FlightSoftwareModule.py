from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import macros
import numpy as np
from skyfield.api import load
from skyfield.framelib import itrs
from datetime import timedelta, datetime, timezone

import Quaternions as quat

class FlightSoftware(sysModel.SysModel):
    def __init__(self, config):
        super(FlightSoftware, self).__init__()
        self.ModelTag = "flight_software"

        # Create readers for the star tracker, IMU, magnetometer, reaction wheel messages and spacecraft positioning. Spacecraft positioning used to emulate GPS input
        self.starTrackerMsgIn = messaging.STSensorMsgReader() 
        self.imuMsgIn = messaging.IMUSensorMsgReader()
        self.rwSpeedMsgIn = messaging.RWSpeedMsgReader()
        self.magMsgIn = messaging.TAMSensorMsgReader() # Three-Axis Magnetometer
        self.scStateIn = messaging.SCStatesMsgReader()
        self.earthStateInMsg = messaging.SpicePlanetStateMsgReader()
        
        # setup reaction wheel output messages
        self.rwMotorTorqueOutMsg = messaging.ArrayMotorTorqueMsg()
        self.rwMotorTorquePayload = messaging.ArrayMotorTorqueMsgPayload()
        self.torque_vals = np.zeros(36) # initialize RW torque input array
        
        # setup magnetorquer output messages
        self.magTorqueOutMsg = messaging.MTBCmdMsg()
        self.magTorquePayload = messaging.MTBCmdMsgPayload()
        self.mag_torques = np.zeros(36) # initialize MTB torque input array
        
        self.rwInertia = config["rw_Inertia"] # reaction wheel inertia (scalar)
        self.updateTime = config["fsw_update_time"]
        self.use_filter = config["use_filter"]
        self.guidance_mode = config["guidance_mode"] # True or False, set tracking mode to slowly slew satellite over time to emulate target tracking mode
        self.error_filter = [] # used for tracking and graphing filter error (estimated error based on filter state estimates)
        self.target_history = [] # only used if self.guidance_mode is not None
        self.time_zero = 0 # initialized in sim main, used to keep track of GPS time
        self.omega_earth = 7.2921150e-5 # sidereal rotation rate of Earth for propogation calculations [rad/s]. Approximation suffices for the calculations we're doing
        self.r_earth = 4.07e7 # approximate earth radius for line-of-sight (LOS) calculations
        self.activate_on_overpass = config["activate_on_overpass"] # determines whether overpass time should be calculated, and an activation time for control systems set
        self.use_variable_gain = config["use_variable_gain"] # LQR tuning with or without integrator terms for steady state error corrections
        
        self.q_target = np.array([0,0,0,1]) # attribute initialization, set to real value in sim main
        omega_target_rpm = np.array([0.0, 0.0, 0.0]) # [RPM]
        self.omega_target = omega_target_rpm * 2*np.pi/60 # convert to [rad/s]
        
        self.use_skyfield = config["use_skyfield"]
        if self.use_skyfield or self.activate_on_overpass:
            self.skyfield_timescale = load.timescale()
            self.skyfield_EOP = itrs
        
        # self.maxTorque = 0.01 # maximum torque output of reaction wheel (this is just to properly simulate, doesn't currently reflect the real-world behavior of OreSat reaction wheels)
        self.maxTorque = 0.001 # maximum torque output of reaction wheel (this is just to properly simulate, doesn't currently reflect the real-world behavior of OreSat reaction wheels)
        self.maxSpeed = 10000 * macros.RPM # converts RPM to [rad/s]
        
        # Select the spacecraft pointing reference (which axis/sensor defines boresight):
        # Modes are ST (Star Tracker, +x on body), SC (Selfie Camera, +z on body), CFC (Cirrus Flux Camera, -z on body)    
        self.pointing_reference = config["pointing_reference"]
        self.q_90_rot = quat.axis_angle_to_quaternion([0,1,0], -90) # translate star tracker targets to +z side of satellite by rotating by 90 degrees CW about the y axis
        self.q_180_rot = quat.axis_angle_to_quaternion([1,0,0], -180) # translate CFC targets to +z side/viewpoint of satellite. Chose rotation about x axis for this one so that satellite +x facing doesn't change in guidance functions
        
        self.last_skyfield_frame = None # stores last Skyfield ECI_2_ECEF rotation matrix
        self.last_frame_time = None # stores last time Skyfield ECI_2_ECEF rotation matrix was updated
        self.skyfield_rate = int(1/self.updateTime) # convert update rate to ticks
        self.ST_rate_check = int(config["ST_update_rate"]/self.updateTime) # how many fsw 'ticks' between star tracker update
        self.tracker_count = 0
        self.ticks = 0
        
        self.omega_desired_prev = np.zeros(3)
    
    def set_time_zero_from_iso_utc(self, iso_utc: str): # used to initialize ephemeris start time for GPS timestamp emulation. Only used in simulation software, not flight software.
        s = iso_utc.replace("Z", "+00:00") # Accepts formats "2026-02-10T00:00:00Z" or "2026-02-10T00:00:00+00:00"
        self.time_zero = datetime.fromisoformat(s).astimezone(timezone.utc)
        
    def Reset(self, currentTimeNanos): # required by Basilisk even if Reset does nothing
        pass
        # print(f"({self.ModelTag}) Reset called at {currentTimeNanos * macros.1e-9:.2f} s") # commented out to remove unnecessary printing every simulation
    
    def UpdateState(self, currentTimeNanos):
        self.ticks += 1
        
        '''
        The following section gathers all sensor and frame states.
        '''
        
        ######### GATHER SYSTEM STATES AND CALCULATE ERROR QUATERNION #########
        # if self.imuMsgIn.isWritten():
        #     self.imuMsg = self.imuMsgIn()
        #     omega = np.asarray(self.imuMsg.AngVelPlatform) # explicitly stored as numpy array for filter operations
        # if self.rwSpeedMsgIn.isWritten():
        #     self.rwSpeedMsg = self.rwSpeedMsgIn()
        #     wheelSpeeds = self.rwSpeedMsg.wheelSpeeds
        # if self.magMsgIn.isWritten():
        #     self.magMsg = self.magMsgIn()
        #     B = np.asarray(self.magMsg.tam_S)
        # if self.starTrackerMsgIn.isWritten():
        #     self.starTrackerMsg = self.starTrackerMsgIn()
        #     q_star_tracker = self.starTrackerMsg.qInrtl2Case  # Star Tracker measurement [qs, q1, q2, q3]
        #     q_star_tracker = quat.to_scalar_last(q_star_tracker) # convert Basilisk quaternion to scalar last: [q1, q2, q3, qs]
        # if self.scStateIn.isWritten():
        #     scState = self.scStateIn()
        #     r_CN_N = scState.r_CN_N # spacecraft inertial vector (position from COM) from origin (Earth) in ECI frame.
        #     v_CN_N = scState.v_CN_N # spacecraft velocity vector (position from COM) from origin (Earth) in ECI frame.
        # if self.earthStateInMsg.isWritten():
        #     earthState = self.earthStateInMsg()
        #     true_ECI_2_ECEF = np.asarray(earthState.J20002Pfix) # sim-internal transform matrix from ECI to ECEF frame
        #     r_ECEF = true_ECI_2_ECEF @ r_CN_N # Convert Earth-centered inertial to ECEF to emulate GPS data. Technical name is r_CE_E, using r_ECEF for readability
        #     v_ECEF = true_ECI_2_ECEF @ v_CN_N # Spacecraft orbital velocity vector. Convert to ECEF to emulate GPS data.
            
        #######################################################################
        
        self.imuMsg = self.imuMsgIn()
        omega = np.asarray(self.imuMsg.AngVelPlatform) # explicitly stored as numpy array for filter operations

        self.rwSpeedMsg = self.rwSpeedMsgIn()
        wheelSpeeds = self.rwSpeedMsg.wheelSpeeds

        self.magMsg = self.magMsgIn()
        B = np.asarray(self.magMsg.tam_S)

        if self.ticks % self.ST_rate_check == 0: # emulate slower star tracker update rate
            self.starTrackerMsg = self.starTrackerMsgIn()
            q_star_tracker = self.starTrackerMsg.qInrtl2Case  # Star Tracker measurement [qs, q1, q2, q3]
            q_star_tracker = quat.to_scalar_last(q_star_tracker) # convert Basilisk quaternion to scalar last: [q1, q2, q3, qs]
        else:
            q_star_tracker = None
        
        scState = self.scStateIn()
        r_CN_N = scState.r_CN_N # spacecraft inertial vector (position from COM) from origin (Earth) in ECI frame.
        v_CN_N = scState.v_CN_N # spacecraft velocity vector (position from COM) from origin (Earth) in ECI frame.

        earthState = self.earthStateInMsg()
        true_ECI_2_ECEF = np.asarray(earthState.J20002Pfix) # sim-internal transform matrix from ECI to ECEF frame
        r_ECEF = true_ECI_2_ECEF @ r_CN_N # Convert Earth-centered inertial to ECEF to emulate GPS data. Technical name is r_CE_E, using r_ECEF for readability
        v_ECEF = true_ECI_2_ECEF @ v_CN_N # Spacecraft orbital velocity vector. Convert to ECEF to emulate GPS data.
        
        #######################################################################
        
        # SEND DATA OUT
        
        # RECEIVE BACK ACTUATOR COMMANDS FROM C3
        RW_torques = np.array([0,0,0,0]) # PLACEHOLDER
        MTB_torques = np.array([0,0,0]) # PLACEHOLDER
        
        # COMMAND TORQUES
        if np.any(RW_torques):
            self.command_wheel_torques(currentTimeNanos, RW_torques, wheelSpeeds)
            self.rwMotorTorquePayload.motorTorque = self.torque_vals
            self.rwMotorTorqueOutMsg.write(self.rwMotorTorquePayload, currentTimeNanos, self.moduleID)
        elif np.any(MTB_torques):
            self.command_MTB_torques(MTB_torques, currentTimeNanos)
            
        if not np.any(RW_torques): # if controller should be off, simulate wheel shutdown by sending required torques to null wheelspeeds. This is only required in simulation, as wheel cogging will have the same affect uncommanded for the real satellite.
            # Zero wheel torques    
            wheel_torque = [0]*4
            for i in range(4):
                wheel_torque[i] = (-self.rwInertia * wheelSpeeds[i] / self.updateTime)/10 # divide by 100 to prevent hysteresis and uncontrolled flipping.
            self.command_wheel_torques(currentTimeNanos, wheel_torque, wheelSpeeds)
        if not np.any(MTB_torques):
            # Zero MTB dipoles
            self.mag_torques[:] = 0.0
            self.magTorquePayload.mtbDipoleCmds = self.mag_torques
            self.magTorqueOutMsg.write(self.magTorquePayload, currentTimeNanos, self.moduleID)
    
    def command_MTB_torques(self, desired_torque, currentTimeNanos):
        self.mag_torques[:3] = desired_torque
        self.magTorquePayload.mtbDipoleCmds = self.mag_torques
        self.magTorqueOutMsg.write(self.magTorquePayload, currentTimeNanos, self.moduleID)
        
    def command_wheel_torques(self, currentTimeNanos, wheel_torque, wheelSpeeds): # send commanded torque values to reaction wheels
        self.check_torque_vals(wheel_torque, wheelSpeeds) # ensure none of the torque values exceed max torque or accelerate wheel past max RPM in either direction and write to self.torque_vals
        self.rwMotorTorquePayload.motorTorque = self.torque_vals
        self.rwMotorTorqueOutMsg.write(self.rwMotorTorquePayload, currentTimeNanos, self.moduleID)
          
    def check_torque_vals(self, wheel_torque, rwSpeeds): # ensure torque does not exceed maxTorque and that wheel speed does not exceed maxSpeed by the beginning of next step
        for i in range(len(self.torque_vals[:4])):
            projected_speed = rwSpeeds[i] + (wheel_torque[i]/self.rwInertia) * self.updateTime # predicted speed at requested torque after next time step
            if abs(projected_speed) > self.maxSpeed: # Clamp torque if it would cause overspeed
                speed_sign = np.sign(rwSpeeds[i]) if rwSpeeds[i] != 0 else np.sign(wheel_torque[i])
                required_torque = (speed_sign * self.maxSpeed - rwSpeeds[i]) * self.rwInertia / self.updateTime
                self.torque_vals[i] = max(-self.maxTorque, min(required_torque, self.maxTorque))
            else:  # Otherwise clamp to max torque bounds
                self.torque_vals[i] = max(-self.maxTorque, min(wheel_torque[i], self.maxTorque))
    
