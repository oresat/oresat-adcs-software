from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import macros
import numpy as np
from ADCS_Discrete_State_Space_Calculator import get_RW_gain_matrix
# from MTB_LQR_Discrete_Gains_Calculator import get_MTB_gain_matrix
import Quaternions as quat
from Kalman_Filter import Multiplicative_Extended_Kalman_Filter
from sys import exit

class FlightSoftware(sysModel.SysModel):
    def __init__(self, config):
        super(FlightSoftware, self).__init__()
        self.ModelTag = "flight_software"

        # Create readers for the star tracker, IMU, magnetometer, and reaction wheel messages
        self.starTrackerMsgIn = messaging.STSensorMsgReader() 
        self.imuMsgIn = messaging.IMUSensorMsgReader()
        self.rwSpeedMsgIn = messaging.RWSpeedMsgReader()
        self.magMsgIn = messaging.TAMSensorMsgReader()
        
        # setup reaction wheel output messages
        self.rwMotorTorqueOutMsg = messaging.ArrayMotorTorqueMsg()
        self.rwMotorTorquePayload = messaging.ArrayMotorTorqueMsgPayload()
        self.torque_vals = np.zeros(36) # initialize RW torque input array
        
        # setup magnetorquer output messages
        self.magTorqueOutMsg = messaging.MTBCmdMsg()
        self.magTorquePayload = messaging.MTBCmdMsgPayload()
        self.mag_torques = np.zeros(36) # initialize MTB torque input array
        
        self.G_pinv = -np.linalg.pinv(config["G"]) # pseudo inverse matrix for torque calculations. Negated because of Basilisk conventions (I think)
        self.rwInertia = config["rw_Inertia"] # reaction wheel inertia (scalar)
        self.satInertia = config["J"] # satellite inertia tensor (matrix)
        self.updateTime = config["fsw_update_time"]
        self.output_states = config["print_states"] # output state messages or not for debugging
        self.use_filter = config["use_filter"]
        self.target_tracking = config["tracking_mode_active"] # True or False, set tracking mode to slowly slew satellite over time to emulate target tracking mode
        self.crashTheKernel = False # intentional exit to catch errors. Crashes the kernel because of SWIG. 
        self.error_filter = [] # used for tracking and graphing filter error (estimated error based on filter state estimates)
        self.target_history = [] # only used if self.target_tracking = True
        
        self.q_target = np.array([0,0,0,1]) # attribute initialization, set to real value in sim main
        omega_target_rpm = np.array([0.0, 0.0, 0.0])
        self.omega_target = omega_target_rpm * 2*np.pi/60 # convert to rad/s
        
        self.maxTorque = 0.01 # maximum torque output of reaction wheel (this is just to properly simulate, doesn't currently reflect the real-world behavior of OreSat reaction wheels)
        self.maxSpeed = 10000 * macros.RPM # converts RPM to [rad/s]
        self.bangbang_rate = 0.07 # max rotation rate of bang bang controller (0.07 rad/s ~ 4 deg/s)
        self.thermal_spin_rpm = 1.0 # thermal spin rate about the z-axis (body frame)
        self.controllerStartTime = 0 # time at which controller should begin taking control [seconds]
        
        use_integrator = False # use gain matrix with integrator or without
        # LQR_max_error = 0.02 # VERY SLOWED TUNING FOR ASNYCHRONOUS SENSOR FILTERING
        # LQR_max_rate = 0.001
        # LQR_max_error = 0.01 # SLOWED TUNING FOR ASNYCHRONOUS SENSOR FILTERING
        # LQR_max_rate = 0.002
        # LQR_max_error = 0.01 # GOOD TUNING FOR STANDARD LQR WITHOUT FILTERING
        # LQR_max_rate = 0.003
        LQR_max_error = 0.001 # FAST TUNING FOR TRACKING
        LQR_max_rate = 0.1
        self.K_RW = get_RW_gain_matrix(self.satInertia, self.updateTime, LQR_max_error, LQR_max_rate, use_integrator)
        # self.K_MTB = get_MTB_gain_matrix(self.satInertia, self.updateTime, LQR_max_error, LQR_max_rate, config["orbital_period"])
        self.actuator_mode = config["actuator_mode"] # activates either Reaction Wheels ("RW") or Magnetorquers ("MAG")
        self.mission_mode = config["mission_mode"]
        self.slewMode = "slew" # only used for sliding mode bang-bang controller. Can be "slew" for large-angle rotations or "precise" for fine-pointing operations
        
        # gains for variable-gain sliding mode controller
        self.fastGain = get_RW_gain_matrix(self.satInertia, self.updateTime, 0.01, 0.003, use_integrator)
        self.slowGain = get_RW_gain_matrix(self.satInertia, self.updateTime, 0.03, 0.001, use_integrator)
        self.gainMode = "fast"
        
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
        self.EKF = Multiplicative_Extended_Kalman_Filter(self.updateTime, config["P_ST_0"], config["sigma_ST"], config["P_b0"], config["sigma_gyro"], config["sigma_bias"])
        
        self.ST_rate_check = int(config["ST_update_rate"]/self.updateTime) # how many fsw 'ticks' between star tracker update
        self.tracker_count = 0
        self.ticks = 0
        
        self.rotate = quat.axis_angle_to_quaternion([0,1,0], -0.02)
        
    def Reset(self, currentTimeNanos):
        pass
        # print(f"({self.ModelTag}) Reset called at {currentTimeNanos * macros.NANO2SEC:.2f} s") # commented out to remove unnecessary printing every execution
        
    def UpdateState(self, currentTimeNanos):
        if self.crashTheKernel == True: # This method allows error message printing  *jank intensifies*
            exit()
            
        self.ticks += 1
        
        if self.target_tracking == True:
            self.q_target = quat.quat_mult(self.rotate, self.q_target)
            self.target_history.append(self.q_target)
        
        ######### GATHER SYSTEM STATES AND CALCULATE ERROR QUATERNION #########
        if self.imuMsgIn.isWritten():
            self.imuMsg = self.imuMsgIn()
            omega = self.imuMsg.AngVelPlatform
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
        
        if self.use_filter: # simulate asynchronous MEKF
            if (currentTimeNanos == 0):
                q = quat.quat_mult(self.q_90_rot, q_star_tracker) # convert star tracker output to nominal body frame (+z with selfie cam)
                self.EKF.q = q # initialize filter
            elif (self.ticks % self.ST_rate_check == 0): # account for star tracker update rate
                self.tracker_count += 1
                q_st_rotated = quat.quat_mult(self.q_90_rot, q_star_tracker) # convert star tracker output to nominal body frame (+z with selfie cam)
                q, omega = self.EKF.update(omega, q_st_rotated)
            else: # else only propagate estimate with body rates if no star tracker update available
                q, omega= self.EKF.update(omega)
        else: # send sensor data directly to the controller without filtering
            q = quat.quat_mult(self.q_90_rot, q_star_tracker) # convert star tracker output to nominal body frame (+z with selfie cam)
            
        q_error = quat.quat_error(self.q_target, q) # get error quaternion, this function automatically sanitizes by performing normalization and hemisphere checks
        q_error = quat.hemi(q_error) # only apply hemisphere check once after determining error quaternion to maintain associativity across hermisphere boundaries
        self.error_filter.append(q_error) # save estimated (filtered) attitude error for plotting after conclusion of sim execution
        
        ######################### CONTROL LOGIC ###############################    
        if (currentTimeNanos * macros.NANO2SEC >= self.controllerStartTime): # turn controller on at specified time and check control mode for either Reaction Wheel or Magnetorquer (MTB) control
            if self.actuator_mode == "RW":
                if self.mission_mode == "POINTING":
                    desired_torque = self.quaternion_controller(q_error, omega) # compute desired 3-axis torque from controller (standard LQR controller)
                    # desired_torque = self.sliding_bangbang_controller(q_error, omega, currentTimeNanos) # compute desired 3-axis torque from controller (bang-bang sliding mode controller)
                    # desired_torque = self.variable_gain_controller(q_error, omega, currentTimeNanos) # compute desired 3-axis torque from controller (sliding-mode variable gain controller)
                    wheel_torque = self.convert_torque_to_wheels(desired_torque) # convert desired 3-axis torque to inputs for 4 wheels
                    self.command_wheel_torques(currentTimeNanos, wheel_torque, wheelSpeeds) # Write the payload to reaction wheels
                elif self.mission_mode == "THERMAL_REORIENT": # can only be set by first part of passive thermal spin controller
                    desired_torque = self.quaternion_controller(q_error, omega) # compute desired 3-axis torque from controller
                    wheel_torque = self.convert_torque_to_wheels(desired_torque) # convert desired 3-axis torque to inputs for 4 wheels
                    self.command_wheel_torques(currentTimeNanos, wheel_torque, wheelSpeeds) # Write the payload to reaction wheels
                    if (quat.error_angle(q_error) <= 0.1 and np.all(np.abs(omega) < 1e-6)):
                        #zero wheel speeds?
                        self.actuator_mode = "MAG" # switch to MTB's for spinup
                        self.mission_mode = "SPINUP" 
                        print(f"SWITCHING TO MAGNETORQUER SPINUP AT {currentTimeNanos*macros.NANO2SEC} SECONDS, {omega}")
                else:
                    print("ERROR: Unknown mission mode specified", flush = True)
                    self.crashTheKernel = True
            
            elif self.actuator_mode == "MAG":
                if self.mission_mode == "DETUMBLE":
                    desired_torque = self.detumble_gain/(np.linalg.norm(B)**2)*np.cross(omega, B) # detumble controller as defined by Markley & Crassidis
                    self.command_MTB_torques(desired_torque, currentTimeNanos) # Write the payload to magnetorquers
                elif self.mission_mode == "THERMAL_SPIN":
                    desired_torque = self.detumble_gain/(np.linalg.norm(B)**2)*np.cross(omega, B) # detumble controller as defined by Markley & Crassidis
                    self.command_MTB_torques(desired_torque, currentTimeNanos) # Write the payload to magnetorquers
                    if (np.all(np.abs(omega) < 1e-4)):
                        self.actuator_mode = "RW" # switch to reaction wheels for thermal reorientation
                        self.mission_mode = "THERMAL_REORIENT"
                        print(f"SWITCHING TO REACTION WHEEL REORIENTATION AT {currentTimeNanos*macros.NANO2SEC} SECONDS")
                # elif self.mission_mode ==  "POINTING":
                #     # body_torques = self.mag_LQR_controller(q_error, omega) # calculate body-frame torques
                #     # desired_torque = np.cross(b_field, body_torques)/np.linalg.norm(b_field)**2 #project torques into dipole moments
                #     # self.command_MTB_torques(desired_torque, currentTimeNanos) # Write the payload to magnetorquers
                #     tau_des = self.mag_LQR_controller(q_error, omega)          # torque request
                #     B2 = B @ B
                #     m_cmd = np.zeros(3) if B2 < (5e-6)**2 else np.cross(B, tau_des) / B2  # A·m²
                #     self.command_MTB_torques(m_cmd, currentTimeNanos)
                elif self.mission_mode == "SPINUP": # spinup satellite for thermal spin about the axis
                    if (omega[2] < self.thermal_spin_rpm*2*np.pi/60): # while satellite is spinning slower than set rate about the z axis, spin up
                        tau_des = [0,0,1] # spin about the z axis
                        m = np.cross(B, tau_des) / (B @ B)
                        self.command_MTB_torques(m, currentTimeNanos)
                else:
                    print("ERROR: Unknown mission mode specified", flush = True)
                    self.crashTheKernel = True
            
        if self.output_states:
            q_reconstruct = quat.quat_mult(quat.quat_conjugate(q_error), q)
            print("\nTime: ", currentTimeNanos * macros.NANO2SEC)
            print("Current quaternion: ", q)
            print("Raw Star Tracker output: ", q_star_tracker)
            print("Error quaternion: ", q_error)
            print("Axis of rotation: ", quat.quat_to_axis(q_error))
            print("Current angle error: ", quat.error_angle(q_error))
            print("Reconstructed target: ", q_reconstruct)
            print("Actual target:        ", self.q_target) # this checks that the error quaternion is properly defined (it is)
            print("Current wheel speeds: ", wheelSpeeds[:4])
            print("Body rates: ", omega)
            print("Desired torque: ", desired_torque)
            print("Wheel Torque: ", wheel_torque)
    
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
          
    def convert_torque_to_wheels(self, torque_array): # convert 3-axis torque request to üyramid configuration reaction wheel output
        if (self.G_pinv.shape[1] != np.shape(torque_array)[0]):
            print(f"\n\nMANUAL ERROR: Array shapes do not match. Got G_pinv shape [1] {self.G_pinv.shape[1]} and torque_array shape [0] {np.shape(torque_array)[0]}", flush = True)  # this doesn't work because of Basilisk stuff, kernel crashes before flushing output buffer
        return self.G_pinv @ torque_array
    
    def check_torque_vals(self, wheel_torque, rwSpeeds): # ensure torque does not exceed maxTorque and that wheel speed does not exceed maxSpeed by the beginning of next step
        
        wheel_torque = wheel_torque * 0.05
        
        for i in range(len(self.torque_vals[:4])):
            projected_speed = rwSpeeds[i] + (wheel_torque[i]/self.rwInertia) * self.updateTime # predicted speed at requested torque after next time step
            if abs(projected_speed) > self.maxSpeed: # Clamp torque if it would cause overspeed
                speed_sign = np.sign(rwSpeeds[i]) if rwSpeeds[i] != 0 else np.sign(wheel_torque[i])
                required_torque = (speed_sign * self.maxSpeed - rwSpeeds[i]) * self.rwInertia / self.updateTime
                self.torque_vals[i] = max(-self.maxTorque, min(required_torque, self.maxTorque))
            else:  # Otherwise clamp to max torque bounds
                self.torque_vals[i] = max(-self.maxTorque, min(wheel_torque[i], self.maxTorque))
    
    # def mag_LQR_controller(self, q_error, omega):
    #     x = np.concatenate((q_error[:3], omega)) # assemble state vector
    #     return -self.K_MTB @ x
    
    def quaternion_controller(self, q_error, omega):
        x = np.concatenate((q_error[:3], omega)) # assemble state vector
        return -self.K_RW @ x # invert sign for control
    
    def slew_mode_check(self, error_angle_degrees, currentTimeNanos): # check which control mode to use for sliding-mode controllers. 10 degree buffer zone to prevent hysteresis
            if (error_angle_degrees < 40 and self.slewMode == "slew"): # switch to precision guidance mode
                self.slewMode = "precise"
                print(f"Mode switched to precise with remaining error of {error_angle_degrees} degrees at {currentTimeNanos*macros.NANO2SEC} seconds", flush = True)
            elif (error_angle_degrees > 50 and self.slewMode == "precise"): # switch to slew mode 
                self.slewMode = "slew"
                print(f"Mode switched to slew with remaining error of {error_angle_degrees} degrees at {currentTimeNanos*macros.NANO2SEC} seconds", flush = True)
    
    def bang_bang_controller(self, q_error, omega):
        axis = -quat.quat_to_axis(q_error) # determine axis of rotation
        omega_target = axis*self.bangbang_rate
        omega_error = omega_target - omega
        axis_torque = self.satInertia @ omega_error/self.updateTime # tau = I*omega/dt
        return axis_torque # negate to account for reaction wheel opposite reactions
    
    def sliding_bangbang_controller(self, q_error, omega, currentTimeNanos):            
        error_angle_degrees = quat.error_angle(q_error) # get minimum error angle (in degrees)
        self.slew_mode_check(error_angle_degrees, currentTimeNanos) # switch modes based on current error

        if self.slewMode == "precise":
            return self.quaternion_controller(q_error, omega)
        elif self.slewMode == "slew":
            return self.bang_bang_controller(q_error, omega)
        
    def variable_gain_controller(self, q_error, omega, currentTimeNanos):
        error_angle_degrees = quat.error_angle(q_error) # get minimum error angle (in degrees)
        if (error_angle_degrees < 1 and self.gainMode == "fast"): # switch to precision guidance mode
            self.gainMode = "slow"
            print(f"Gain mode switched to slow with remaining error of {error_angle_degrees} degrees at {currentTimeNanos*macros.NANO2SEC} seconds", flush = True)
        elif (error_angle_degrees > 5 and self.gainMode == "slow"): # switch to slew mode 
            self.gainMode = "fast"
            print(f"Gain mode switched to fast with remaining error of {error_angle_degrees} degrees at {currentTimeNanos*macros.NANO2SEC} seconds", flush = True)
        
        x = np.concatenate((q_error[:3], omega)) # assemble state vector
        
        if self.gainMode == "fast":
            return -self.fastGain @ x # invert sign for control
        elif self.gainMode == "slow":
            return -self.slowGain @ x # invert sign for control
        