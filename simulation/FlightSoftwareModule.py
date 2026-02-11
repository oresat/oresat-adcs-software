from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import macros
import numpy as np
from ADCS_Discrete_State_Space_Calculator import get_RW_gain_matrix
import Quaternions as quat
# import Guidance_Functions as guidance
from Kalman_Filter import Multiplicative_Extended_Kalman_Filter
from skyfield.api import load
from skyfield.framelib import itrs
from datetime import datetime, timezone, timedelta
from sys import exit

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
        
        self.G = config["G"]
        self.G_transpose = self.G.T # save repeated calculations each iteration
        self.G_pinv = -np.linalg.pinv(self.G) # pseudo inverse matrix for torque calculations. Negated because of Basilisk conventions (I think)
        self.rwInertia = config["rw_Inertia"] # reaction wheel inertia (scalar)
        self.satInertia = config["J"] # satellite inertia tensor (matrix)
        self.updateTime = config["fsw_update_time"]
        self.output_states = config["print_states"] # output state messages (or not) for debugging
        self.use_filter = config["use_filter"]
        self.tracking_mode = config["tracking_mode"] # True or False, set tracking mode to slowly slew satellite over time to emulate target tracking mode
        self.crashTheKernel = False # intentional exit to catch errors. Crashes the kernel because of SWIG. 
        self.error_filter = [] # used for tracking and graphing filter error (estimated error based on filter state estimates)
        self.target_history = [] # only used if self.tracking_mode is not None
        self.time_zero = 0 # initialized in sim main, used to keep track of GPS time
        
        self.q_target = np.array([0,0,0,1]) # attribute initialization, set to real value in sim main
        omega_target_rpm = np.array([0.0, 0.0, 0.0]) # [RPM]
        self.omega_target = omega_target_rpm * 2*np.pi/60 # convert to [rad/s]
        
        # constants used for GPS-to-ECEF conversion
        a = 6378137.0 # WGS-84 constant: a = semi-major axis
        e2 = 0.0066943799901413165 # WGS-84 constant: e^2 = flattening
        target_lat = config["target_lat"]
        target_lon = config["target_lon"]
        target_height = config["target_height"]
        self.ECEF_target = self.GPS_to_ECEF(target_lat, target_lon, target_height, a, e2) # convert GPS coordinates to ECEF coordinates
        
        self.use_skyfield = config["use_skyfield"]
        if self.use_skyfield:
            self.skyfield_timescale = load.timescale()
            # self.skyfield_ephemeris = load('de440s.bsp') # UPDATE THIS TO POINT TO ACTUAL FILE || NOT TOO SENSITIVE TO STALE FILES
            self.skyfield_EOP = itrs
        
        self.maxTorque = 0.01 # maximum torque output of reaction wheel (this is just to properly simulate, doesn't currently reflect the real-world behavior of OreSat reaction wheels)
        self.maxSpeed = 10000 * macros.RPM # converts RPM to [rad/s]
        self.bangbang_rate = 0.07 # max rotation rate of bang bang controller (0.07 rad/s ~ 4 deg/s)
        self.thermal_spin_rpm = 1.0 # thermal spin rate about the z-axis (body frame)
        self.controllerStartTime = 0 # time at which controller should activate [seconds]
        
        max_input = 0.00003 # QUALITATIVE value for max torque used by LQR tuning ONLY
        LQR_max_error = 0.01
        LQR_max_rate = 0.002
        self.K_RW = get_RW_gain_matrix(self.satInertia, self.updateTime, LQR_max_error, LQR_max_rate, max_input)
        
        self.control_mode = config["control_mode"]
        self.slewMode = "slew" # only used for sliding mode bang-bang controller. Can be "slew" for large-angle rotations or "precise" for fine-pointing operations
        
        # Select the spacecraft pointing reference (which axis/sensor defines boresight):
        # Modes are ST (Star Tracker, +x on body), SC (Selfie Camera, +z on body), CFC (Cirrus Flux Camera, -z on body)    
        self.pointing = config["pointing_reference"]
        self.q_90_rot = quat.axis_angle_to_quaternion([0,1,0], -90) # translate star tracker targets to +z side of satellite
        self.q_180_rot = quat.axis_angle_to_quaternion([0,1,0], -180) # translate CFC targets to +z side/viewpoint of satellite
        
        # Controller gains
        Jmin = np.min(np.linalg.eigvals(self.satInertia)) # maximum principal moment of inertia (Markley & Crassidis defines this with the minimum principal moment of inertia, but maximum works better???)
        self.detumble_gain = 4*np.pi/config["orbital_period"]*(1+np.sin(config["orbital_inclination"]*2*np.pi/180))*Jmin # gain based on minimal principal moment of inertia as defined in Markley & Crassidis
        
        # Kalman filter object to store filter states and sensor values
        self.gyro_bias_drift_rate = 0.015 * macros.D2R # [rad/s/K] additional bias drift dependent on difference between current and reference (25 C) temperatures
        self.EKF = Multiplicative_Extended_Kalman_Filter(config["P_ST_0"], config["sigma_ST"], config["P_b0"], config["sigma_gyro"], config["sigma_bias"])
        
        self.last_skyfield_frame = None # stores last Skyfield ECI_2_ECEF rotation matrix
        self.skyfield_rate = int(1/self.updateTime) # convert update rate to ticks
        self.ST_rate_check = int(config["ST_update_rate"]/self.updateTime) # how many fsw 'ticks' between star tracker update
        self.tracker_count = 0
        self.ticks = 0
        
        self.rotate = quat.axis_angle_to_quaternion([0,1,0], -0.02) # for target tracking emulation REMOVE
        
        self.omega_desired_prev = np.zeros(3)
        
        max_input_mag = 0.3 # QUALITATIVE value for max torque used by LQR tuning ONLY
        LQR_max_error_mag = 0.05
        LQR_max_rate_mag = 0.000008
        self.K_MAG = get_RW_gain_matrix(self.satInertia, self.updateTime, LQR_max_error_mag, LQR_max_rate_mag, max_input_mag)
        self.mag_torque_integral = 0
        
    def Reset(self, currentTimeNanos): # required by Basilisk even if Reset does nothing
        pass
        # print(f"({self.ModelTag}) Reset called at {currentTimeNanos * macros.1e-9:.2f} s") # commented out to remove unnecessary printing every simulation
        
    def UpdateState(self, currentTimeNanos):
        if self.crashTheKernel == True: # This method allows error message printing  *jank intensifies*
            exit()
            
        self.ticks += 1
        
        '''
        The following section gathers all sensor and frame states.
        '''
        
        ######### GATHER SYSTEM STATES AND CALCULATE ERROR QUATERNION #########
        if self.imuMsgIn.isWritten():
            self.imuMsg = self.imuMsgIn()
            omega = np.asarray(self.imuMsg.AngVelPlatform) # explicitly stored as numpy array for filter operations
        if self.rwSpeedMsgIn.isWritten():
            self.rwSpeedMsg = self.rwSpeedMsgIn()
            wheelSpeeds = self.rwSpeedMsg.wheelSpeeds
        if self.magMsgIn.isWritten():
            self.magMsg = self.magMsgIn()
            B = np.asarray(self.magMsg.tam_S)
        if self.starTrackerMsgIn.isWritten():
            self.starTrackerMsg = self.starTrackerMsgIn()
            q_star_tracker = self.starTrackerMsg.qInrtl2Case  # Star Tracker measurement [qs, q1, q2, q3]
            q_star_tracker = quat.to_scalar_last(q_star_tracker) # convert Basilisk quaternion to scalar last: [q1, q2, q3, qs]
        if self.scStateIn.isWritten():
            scState = self.scStateIn()
            r_CN_N = scState.r_CN_N # spacecraft inertial vector (position from COM) from origin in ECI frame. Origin is sun in Basilisk
            v_CN_N = scState.v_CN_N # spacecraft velocity vector (position from COM) from origin in ECI frame. Origin is sun in Basilisk
        if self.earthStateInMsg.isWritten():
            earthState = self.earthStateInMsg()
            true_ECI_2_ECEF = np.asarray(earthState.J20002Pfix) # sim-internal transform matrix from ECI to ECEF frame
            r_EN_N = np.asarray(earthState.PositionVector) # Earth position vector from sun (inertial frame) 
            v_EN_N = np.asarray(earthState.VelocityVector) # Earth velocity vector from sun (inertial frame)
        
        '''
        The following section is for attitude estimation if filtering is turned on
        '''
        
        if self.use_filter: # simulate asynchronous MEKF
            if (currentTimeNanos == 0):
                q = quat.quat_mult(self.q_90_rot, q_star_tracker) # convert star tracker output to nominal body frame (+z with selfie cam)
                self.EKF.q = q # initialize filter
                self.EKF.last_omega = omega
            elif (self.ticks % self.ST_rate_check == 0): # account for star tracker update rate  FOR STAR TRACKER OCCLUSION TESTING: and (currentTimeNanos*1e-9 < 30 or currentTimeNanos*1e-9 > 240)
                self.tracker_count += 1
                q_st_rotated = quat.quat_mult(self.q_90_rot, q_star_tracker) # convert star tracker output to nominal body frame (+z with selfie cam)
                q, omega = self.EKF.update(currentTimeNanos*1e-9, omega, q_st_rotated)
            else: # else only propagate estimate with body rates if no star tracker update available
                q, omega = self.EKF.update(currentTimeNanos*1e-9, omega)
        else: # send sensor data directly to the controller without filtering
            q = quat.quat_mult(self.q_90_rot, q_star_tracker) # convert star tracker output to nominal body frame (+z with selfie cam)
        
        '''
        Dynamic guidance functions for target tracking, nadir-pointing, and
        minimum & maximum drag orientation. This is separate from the control
        portion of the code, and just defines the target which is fed into the 
        control algorithms
        '''
        
        if self.tracking_mode is not None:
            q_last = self.q_target # save for tracking rate calculations
            r_CE_N = r_CN_N - r_EN_N # Earth-centered inertial spacecraft position (converted from Sun-centered) allows for emulation of ECEF-vector-converted-GPS coordinates 
            v_ECEF = true_ECI_2_ECEF @ (v_CN_N-v_EN_N) # True, sim-internal Earth-centered inertial spacecraft position (converted from Sun-centered). Convert to ECEF to emulate GPS data.
            
            if self.use_skyfield: # if using skyfield, override the sim-internal frame-transformation matrix with skyfield's
                if self.ticks % self.skyfield_rate == 0 or self.last_skyfield_frame is None: # slow skyfield update for performance reasons. Update rate of 1 second shouldn't introduce more than .005 degrees of error.
                    dt = self.time_zero + timedelta(seconds=currentTimeNanos * 1e-9)
                    t = self.skyfield_timescale.from_datetime(dt)
                    ECI_2_ECEF = self.skyfield_EOP.rotation_at(t) # inertial -> ECEF rotation matrix
                    self.last_skyfield_frame = ECI_2_ECEF
                else:
                    ECI_2_ECEF = self.last_skyfield_frame # if skyfield didn't update this loop, use saved rotation matrix (zero order hold)
            else:
                ECI_2_ECEF = true_ECI_2_ECEF # if not using skyfield, use sim-internal conversion matrix
                
            if self.tracking_mode == "TARGET": # Tracking a static target on the surface of the earth via GPS coordinates        
                r_ECEF = true_ECI_2_ECEF @ r_CE_N # Convert Earth-centered inertial to ECEF to emulate GPS data. Technical name is r_CE_E, using r_ECEF for readability
                target_ECEF = self.ECEF_target - r_ECEF # calculate target vector in ECEF cartesian coordinates
                target_ECEF = target_ECEF/np.linalg.norm(target_ECEF) # normalize to unit vector
                self.target_tracking_quat(target_ECEF, v_ECEF, ECI_2_ECEF) # create orientation quaternion from cartesian target
            elif self.tracking_mode == "NADIR": # Continually face +z nadir (+x as close to ram as possible)
                nadir_vector = true_ECI_2_ECEF @ (-r_CE_N / np.linalg.norm(r_CE_N)) # nadir vector is opposite of vector from earth. Convert to ECEF to emulate GPS data.
                self.target_tracking_quat(nadir_vector, v_ECEF, ECI_2_ECEF) # create orientation quaternion from cartesian target
            elif self.tracking_mode == "MAX_DRAG" or self.tracking_mode == "MIN_DRAG":
                nadir_vector = true_ECI_2_ECEF @ (-r_CE_N / np.linalg.norm(r_CE_N)) # nadir vector is opposite of vector from earth. Convert to ECEF to emulate GPS data.
                self.ram_quaternion(self.tracking_mode, v_ECEF, nadir_vector, ECI_2_ECEF) # calculate ram-facing orientation for either +z or +x axis based on min or max drag
            
            self.target_history.append(self.q_target)
            
            '''
            The following section includes feed-forward terms for target tracking
            to avoid overdamping and to account for gyroscopic effects 
            '''
            
            # feed forward term for angular rate bias
            rotation_quat = quat.quat_error(q_last, self.q_target) # flipped order because of frame conventions for proper signage (body -> target)
            rot_axis = quat.quat_to_axis(rotation_quat)
            rot_angle = quat.error_angle(rotation_quat) * np.pi/180
            omega_desired = rot_axis*(rot_angle/self.updateTime) # set rotation rate for tracking maneuver
            
            # feed forward term to account for stored angular momentum
            alpha_d_B = (omega_desired - self.omega_desired_prev) / self.updateTime # desired acceleration in body frame
            self.omega_desired_prev = omega_desired.copy() # update previous target rate
            H_wheels = self.rwInertia * np.asarray(wheelSpeeds[:4]) @ self.G.T # calculate stored wheel momentum in body frame (resulting in a 3x1 vector of angular momentum axis elements in body frame)
            tau_ff = self.satInertia @ alpha_d_B + np.cross(omega, self.satInertia @ omega + H_wheels) # total feed-forward torque accounting for gyroscopic coupling
        
            omega = omega-omega_desired # set biased omega after using true value to calculate feed forward term
        else:
            tau_ff = 0
        
        '''
        The following section encomposses the control algorithms which
        define actuator output based on state and target
        '''
        
        q_error = quat.quat_error(self.q_target, q) # get error quaternion, this function automatically sanitizes by performing normalization and hemisphere checks
        q_error = quat.hemi(q_error) # only apply hemisphere check once after determining error quaternion to maintain associativity across hemisphere boundaries
        self.error_filter.append(q_error) # save estimated (filtered) attitude error for plotting after conclusion of sim execution
        
        ######################### CONTROL LOGIC ###############################
        if (currentTimeNanos * 1e-9 >= self.controllerStartTime): # turn controller on at specified time
            if self.control_mode == "RW_POINTING":
                desired_torque = self.RW_controller(q_error, omega) # compute desired 3-axis torque from controller (standard LQR controller)
                desired_torque += tau_ff # feedforward torque, only non-zero for tracking mode
                wheel_torque = self.convert_torque_to_wheels(desired_torque) # convert desired 3-axis torque to inputs for 4 wheels
                self.command_wheel_torques(currentTimeNanos, wheel_torque, wheelSpeeds) # Write the payload to reaction wheels
            elif self.control_mode == "THERMAL_REORIENT": # can only be set by first part of passive thermal spin controller
                desired_torque = self.RW_controller(q_error, omega) # compute desired 3-axis torque from controller
                wheel_torque = self.convert_torque_to_wheels(desired_torque) # convert desired 3-axis torque to inputs for 4 wheels
                self.command_wheel_torques(currentTimeNanos, wheel_torque, wheelSpeeds) # Write the payload to reaction wheels
                if (quat.error_angle(q_error) <= 0.1 and np.all(np.abs(omega) < 1e-6)):
                    #zero wheel speeds?
                    self.control_mode = "SPINUP"
                    print(f"SWITCHING TO MAGNETORQUER SPINUP AT {currentTimeNanos*1e-9} SECONDS, {omega}")
            elif self.control_mode == "DETUMBLE":
                desired_torque = self.detumble_gain/(np.linalg.norm(B)**2)*np.cross(omega, B) # detumble controller as defined by Markley & Crassidis
                self.command_MTB_torques(desired_torque, currentTimeNanos) # Write the payload to magnetorquers
            elif self.control_mode == "THERMAL_SPIN":
                desired_torque = self.detumble_gain/(np.linalg.norm(B)**2)*np.cross(omega, B) # detumble controller as defined by Markley & Crassidis
                self.command_MTB_torques(desired_torque, currentTimeNanos) # Write the payload to magnetorquers
                if (np.all(np.abs(omega) < 1e-4)):
                    self.control_mode = "THERMAL_REORIENT"
                    print(f"SWITCHING TO REACTION WHEEL REORIENTATION AT {currentTimeNanos*1e-9} SECONDS")
            elif self.control_mode == "SPINUP": # spinup satellite for thermal spin about the axis
                if (omega[2] < self.thermal_spin_rpm*2*np.pi/60): # while satellite is spinning slower than set rate about the z axis, spin up
                    tau_des = [0,0,1] # spin about the z axis
                    m = np.cross(B, tau_des) / (B @ B)
                    self.command_MTB_torques(m, currentTimeNanos)
            elif self.control_mode == "MTB_POINTING": # Magnetorquer fine pointing controller (experimental)
                tau_des = self.mag_LQR_controller(q_error, omega) # desired 3-axis torque in body frame
                bm = self.b_mat(B)
                k = 1e-8
                m_cmd = np.linalg.inv(bm.T @ bm + k*np.eye(3))@bm.T@tau_des
                m_max = 1
                m_cmd = np.clip(m_cmd, -m_max, m_max)
                
                self.command_MTB_torques(m_cmd, currentTimeNanos)
            elif self.control_mode == "ORBITS":
                pass # mode to simply visualize orbits with large timespans
            else:
                print("ERROR: Unknown mission mode specified", flush = True)
                self.crashTheKernel = True
     
    def b_mat(self, B):
        bx, by, bz = B
        return np.array([
            [0,   bz,  -by],
            [-bz,   0,  bx],
            [by, -bx,   0]
        ])
    
    
    def update_target(self, target_quat):
        if self.pointing == "ST":
            self.q_target = quat.quat_mult(self.q_90_rot, target_quat) # define target in body coordinates
        elif self.pointing == "SC":
            self.q_target = target_quat # target does not require rotation
        elif self.pointing == "CFC":
            self.q_target = quat.quat_mult(self.q_180_rot, target_quat) # define target in body coordinates
    
    def command_MTB_torques(self, desired_torque, currentTimeNanos):
        self.mag_torques[:3] = desired_torque
        self.magTorquePayload.mtbDipoleCmds = self.mag_torques
        self.magTorqueOutMsg.write(self.magTorquePayload, currentTimeNanos, self.moduleID)
        
    def command_wheel_torques(self, currentTimeNanos, wheel_torque, wheelSpeeds): # send commanded torque values to reaction wheels
        self.check_torque_vals(wheel_torque, wheelSpeeds) # ensure none of the torque values exceed max torque or accelerate wheel past max RPM in either direction and write to self.torque_vals
        self.rwMotorTorquePayload.motorTorque = self.torque_vals
        self.rwMotorTorqueOutMsg.write(self.rwMotorTorquePayload, currentTimeNanos, self.moduleID)
          
    def convert_torque_to_wheels(self, torque_array): # convert 3-axis torque request to pyramid configuration reaction wheel output
        if (self.G_pinv.shape[1] != np.shape(torque_array)[0]):
            print(f"\n\nMANUAL ERROR: Array shapes do not match. Got G_pinv shape [1] {self.G_pinv.shape[1]} and torque_array shape [0] {np.shape(torque_array)[0]}", flush = True)  # this doesn't work because of Basilisk stuff, kernel crashes before flushing output buffer
        return self.G_pinv @ torque_array
    
    def check_torque_vals(self, wheel_torque, rwSpeeds): # ensure torque does not exceed maxTorque and that wheel speed does not exceed maxSpeed by the beginning of next step
        for i in range(len(self.torque_vals[:4])):
            projected_speed = rwSpeeds[i] + (wheel_torque[i]/self.rwInertia) * self.updateTime # predicted speed at requested torque after next time step
            if abs(projected_speed) > self.maxSpeed: # Clamp torque if it would cause overspeed
                speed_sign = np.sign(rwSpeeds[i]) if rwSpeeds[i] != 0 else np.sign(wheel_torque[i])
                required_torque = (speed_sign * self.maxSpeed - rwSpeeds[i]) * self.rwInertia / self.updateTime
                self.torque_vals[i] = max(-self.maxTorque, min(required_torque, self.maxTorque))
            else:  # Otherwise clamp to max torque bounds
                self.torque_vals[i] = max(-self.maxTorque, min(wheel_torque[i], self.maxTorque))
    
    def RW_controller(self, q_error, omega):
        x = np.concatenate((q_error[:3], omega)) # assemble state vector
        return -self.K_RW @ x # invert sign for control
    
    def mag_LQR_controller(self, q_error, omega):
        x = np.concatenate((q_error[:3], omega)) # assemble state vector
        return -self.K_MAG @ x # invert sign for control
    
    def GPS_to_ECEF(self, lat, lon, height, a, e2):
        
        sin_lat = np.sin(lat * macros.D2R)
        cos_lat = np.cos(lat * macros.D2R)
        
        N = a/np.sqrt(1-e2*sin_lat**2) # lattitude must be signed for WGS-84
        x = (N+height)*cos_lat*np.cos(lon * macros.D2R)
        y = (N+height)*cos_lat*np.sin(lon * macros.D2R)
        z = (N*(1-e2)+height)*sin_lat

        return np.asarray([x, y, z])

    def target_tracking_quat(self, target_vector, v_ECEF, ECI_2_ECEF):
        '''
        Creates an orientation quaternion forming an orientation based on a target
        vector for the z-facing, and orients the +x facing as close as possible to
        the velocity vector
        
        Used for target tracking and nadir pointing
        '''
        
        R_NE = ECI_2_ECEF.T # rotation matrix from ECEF to ECI
        v_ECI = R_NE @ (v_ECEF/np.linalg.norm(v_ECEF)) # norm velocity vector and convert to ECI
            
        zvec = R_NE @ (target_vector/np.linalg.norm(target_vector)) # norm target vector and convert to ECI
        
        xvec = v_ECI - np.dot(v_ECI, zvec) * zvec # remove component parallel to nadir vector from velocity vector to determine "ram-facing-like" vector
        xvec = xvec/np.linalg.norm(xvec) # norm

        yvec = np.cross(zvec, xvec)
        yvec = yvec/np.linalg.norm(yvec) # norm
        
        # xvec = np.cross(yvec, zvec) # Re-orthogonalize zB to avoid numerical drift
        # xvec = xvec/np.linalg.norm(xvec) # norm
        
        C_BN = np.vstack((xvec, yvec, zvec)) # Create DCM for body orientation in ECI coordinates
        
        target_quat = quat.quat_from_dcm_scalar_last(C_BN) # Convert DCM to quaternion
        self.update_target(target_quat) # update FSW target
        
    def ram_quaternion(self, drag_orientation, v_ECEF, nadir_vec, ECI_2_ECEF):
        '''
        Creates an orientation quaternion forming an orientation based on
        whether maximum or minimum drag is desired. The secondary axis is defined
        as the nadir vector, or as close as possible to it
        '''
        
        R_NE = ECI_2_ECEF.T
        drag_facing = R_NE @ (v_ECEF/np.linalg.norm(v_ECEF)) # norm velocity vector and convert to ECI
        
        nadir_ECI = R_NE @ nadir_vec
        nadir_facing = nadir_ECI - np.dot(nadir_ECI, drag_facing) * drag_facing # remove component parallel to velocity vector from nadir vector to determine "downwards-pointing" vector
        nadir_facing = nadir_facing/np.linalg.norm(nadir_facing) # norm 
        
        if drag_orientation == "MAX_DRAG":
            yvec = np.cross(nadir_facing, drag_facing)
            yvec = yvec/np.linalg.norm(yvec) # norm
            
            C_BN = np.vstack((drag_facing, yvec, nadir_facing)) # Create DCM for body orientation in ECI coordinates
            
        elif drag_orientation == "MIN_DRAG":
            nadir_facing = -nadir_facing # flip vector such that in min_drag mode, the satellite's solar panels (rather than the GPS antenna) are pointing anti-nadir
            yvec = np.cross(drag_facing, nadir_facing)
            yvec = yvec/np.linalg.norm(yvec) # norm
            
            C_BN = np.vstack((nadir_facing, yvec, drag_facing)) # Create DCM for body orientation in ECI coordinates
        
        target_quat = quat.quat_from_dcm_scalar_last(C_BN) # Convert DCM to quaternion
        self.update_target(target_quat) # update FSW target
        
    def set_time_zero_from_iso_utc(self, iso_utc: str):
        s = iso_utc.replace("Z", "+00:00") # Accepts formats "2026-02-10T00:00:00Z" or "2026-02-10T00:00:00+00:00"
        self.time_zero = datetime.fromisoformat(s).astimezone(timezone.utc)