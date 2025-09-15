from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import macros
from array import array # C-style arrays for defining motor torques
import numpy as np
from ADCS_Discrete_State_Space_Calculator import get_gain_matrix
import Quaternions as quat # quaternion operations
from sys import exit

class FlightSoftware(sysModel.SysModel):
    def __init__(self, G_matrix, update_time, rw_Inertia, satInertia, pointing_reference):
        super(FlightSoftware, self).__init__()
        self.ModelTag = "flight_software"

        # Create a reader for the star tracker, IMU and reaction wheel messages
        self.starTrackerMsgIn = messaging.STSensorMsgReader() 
        self.imuMsgIn = messaging.IMUSensorMsgReader()
        self.rwSpeedMsgIn = messaging.RWSpeedMsgReader()
        
        # setup reaction wheel output messages
        self.rwMotorTorqueOutMsg = messaging.ArrayMotorTorqueMsg()
        self.rwMotorTorquePayload = messaging.ArrayMotorTorqueMsgPayload()
        self.torque_vals = np.zeros(36) # initialize torque input array
        
        self.G_pinv = -np.linalg.pinv(G_matrix) # pseudo inverse matrix for torque calculations. Negated because of Basilisk conventions (I think)
        self.G = G_matrix
        self.rwInertia = rw_Inertia
        self.satInertia = satInertia
        self.updateTime = update_time
        self.mode = "slew" # only used for sliding mode controllers
        self.q_target = np.array([0,0,0,1])
        omega_target_rpm = np.array([0.0, 0.0, 0.0])
        self.omega_target = omega_target_rpm * 2*np.pi/60 # convert to rad/s
        
        self.maxTorque = 0.01 # maximum torque output of reaction wheel (this is just to properly simulate, doesn't currently reflect the real-world behavior of OreSat reaction wheels)
        self.maxSpeed = 10000 * macros.RPM # converts RPM to [rad/s]
        self.bangbang_rate = 0.1 # max rotation rate of bang bang controller
        self.output_states = True # output state messages or not for debugging
        self.controllerStartTime = 0 # time at which controller should begin taking control [seconds]
        
        use_integrator = False # use gain matrix with integrator or without
        self.LQR_max_error = 0.05
        self.LQR_max_rate = 0.03
        self.fast_gain = get_gain_matrix(satInertia, update_time, self.LQR_max_error, self.LQR_max_rate, use_integrator)
        self.K = self.fast_gain
        self.mode = "slew"
        
        self.error = [] # used for tracking and graphing error
            
        print(f"\nGain matrix K:\n{self.K}\n")
        
        # Select the spacecraft pointing reference (which axis/sensor defines boresight):
        # Modes are ST (Star Tracker, +x on body), SC (Selfie Camera, +z on body), CFC (Cirrus Flux Camera, -z on body)    
        self.pointing = pointing_reference
        self.q_plusZ_rot = quat.axis_angle_to_quaternion([0,1,0], -90) # rotate star tracker output to +z side of satellite for Selfie Camera
        self.q_minusZ_rot = quat.axis_angle_to_quaternion([0,1,0], 90) # rotate star tracker output to -z side of satellite for Cirrus Flux Camera
        
        self.u_prev = np.zeros(4)
        self.torque_rate_lim = .01

    def Reset(self, currentTimeNanos):
        print(f"({self.ModelTag}) Reset called at {currentTimeNanos * macros.NANO2SEC:.2f} s")
        
    def UpdateState(self, currentTimeNanos):
        
        # gather system states
        if self.starTrackerMsgIn.isWritten():
            self.starTrackerMsg = self.starTrackerMsgIn()
            q_star_tracker = self.starTrackerMsg.qInrtl2Case  # Star Tracker measurement [qs, q1, q2, q3]
            q_star_tracker = quat.to_scalar_last(q_star_tracker) # convert Basilisk quaternion to scalar last: [q1, q2, q3, qs]
            # q_star_tracker = quat.normalize(q_star_tracker) # Not sure if this normalization is necessary at this point. Check Star Tracker output.
            
            # select reference vector for pointing commands
            if self.pointing == "ST":
                q = q_star_tracker
            elif self.pointing == "SC":
                q = quat.quat_mult(self.q_plusZ_rot, q_star_tracker)
            elif self.pointing == "CFC":
                q = quat.quat_mult(self.q_minusZ_rot, q_star_tracker)
            
        if self.imuMsgIn.isWritten():
            self.imuMsg = self.imuMsgIn()
            omega = self.imuMsg.AngVelPlatform
        
        if self.rwSpeedMsgIn.isWritten():
            self.rwSpeedMsg = self.rwSpeedMsgIn()
            wheelSpeeds = self.rwSpeedMsg.wheelSpeeds
            
        q_error = quat.quat_error(self.q_target, q) # get error quaternion, this function automatically sanitizes by performing normalization and hemisphere checks
        q_error = quat.hemi(q_error) # only apply hemisphere check once after determining error quaternion to maintain associativity across hermisphere boundaries
        self.error.append(q_error) # required for plotting after conclusion of sim
        
        # CHECK TORQUES AND COMPARE RESULTS BETWEEN VIEWS TO SEE WHY THEY ARE NOT EQUAL WITH EQUAL ERRORS
        # THEN PASS EXACT SAME INPUT TO CONTROLLER AND SEE IF THAT RESULTS IN SAME BEHAVIOR FOR ONE TIME STEP
        
        if (currentTimeNanos * macros.NANO2SEC >= self.controllerStartTime):
            desired_torque = self.quaternion_controller(q_error, omega) # compute desired 3-axis torque from controller
            # desired_torque = self.sliding_bangbang_quat(q_error, omega, currentTimeNanos) # compute desired 3-axis torque from controller
            
            wheel_torque = self.convert_torque_to_wheels(desired_torque) # convert desired 3-axis torque to inputs for 4 wheels
            self.command_wheel_torques(currentTimeNanos, wheel_torque, wheelSpeeds) # Write the payload
            
            
        if self.output_states:
            q_reconstruct = quat.quat_mult(quat.quat_conjugate(q_error), q)
            print("\nTime: ", currentTimeNanos * macros.NANO2SEC)
            print("Current quaternion: ", q)
            print("Error: ", q_error)
            print("Axis of rotation: ", quat.quat_to_axis(q_error))
            print("Current angle error: ", quat.error_angle(q_error))
            print("Reconstructed target: ", q_reconstruct)
            print("Actual target:        ", self.q_target) # this checks that the error quaternion is properly defined (it is)
            print("Desired torque: ", desired_torque)
            print("Wheel Torque: ", wheel_torque)
            print("Current wheel speeds: ", wheelSpeeds[:4])
        
    def command_wheel_torques(self, currentTimeNanos, wheel_torque, wheelSpeeds): # send commanded torque values to reaction wheels
        self.check_torque_vals(wheel_torque, wheelSpeeds) # ensure none of the torque values exceed max torque or accelerate wheel past max RPM in either direction and write to self.torque_vals
        self.rwMotorTorquePayload.motorTorque = self.torque_vals
        self.rwMotorTorqueOutMsg.write(self.rwMotorTorquePayload, currentTimeNanos, self.moduleID)
          
    def convert_torque_to_wheels(self, torque_array): # convert 3-axis torque request to üyramid configuration reaction wheel output
        if (self.G_pinv.shape[1] != np.shape(torque_array)[0]):
            print(f"\n\nMANUAL ERROR: Array shapes do not match. Got G_pinv shape [1] {self.G_pinv.shape[1]} and torque_array shape [0] {np.shape(torque_array)[0]}", flush = True)  # this doesn't work because of Basilisk stuff, kernel crashes before flushing output buffer
        return self.G_pinv @ torque_array
    
    def check_torque_vals(self, wheel_torque, rwSpeeds): # ensure torque does not exceed maxTorque and that wheel speed does not exceed maxSpeed by the beginning of next step
        
        # wheel_torque = wheel_torque * 0.09
        
        for i in range(len(self.torque_vals[:4])):
            projected_speed = rwSpeeds[i] + (wheel_torque[i]/self.rwInertia) * self.updateTime # predicted speed at requested torque after next time step
            if abs(projected_speed) > self.maxSpeed: # Clamp torque if it would cause overspeed
                speed_sign = np.sign(rwSpeeds[i]) if rwSpeeds[i] != 0 else np.sign(wheel_torque[i])
                required_torque = (speed_sign * self.maxSpeed - rwSpeeds[i]) * self.rwInertia / self.updateTime
                self.torque_vals[i] = max(-self.maxTorque, min(required_torque, self.maxTorque))
            else:  # Otherwise clamp to max torque bounds
                self.torque_vals[i] = max(-self.maxTorque, min(wheel_torque[i], self.maxTorque))
    
    def quaternion_controller(self, q_error, omega):
        omega_error = omega - self.omega_target
        x = np.concatenate((q_error[:3], omega_error)) # assemble state vector
        return -self.K @ x # invert sign for control
    
    def mode_check(self, error_angle_degrees, currentTimeNanos): # check which control mode to use for sliding-mode controllers. 10 degree buffer zone to prevent hysteresis
            if (error_angle_degrees < 40 and self.mode == "slew"): # switch to precision guidance mode
                self.mode = "precise"
                print(f"Mode switched to precise with remaining error of {error_angle_degrees} degrees at {currentTimeNanos*macros.NANO2SEC} seconds", flush = True)
            elif (error_angle_degrees > 50 and self.mode == "precise"): # switch to slew mode 
                self.mode = "slew"
                print(f"Mode switched to slew with remaining error of {error_angle_degrees} degrees at {currentTimeNanos*macros.NANO2SEC} seconds", flush = True)
    
    def bang_bang_controller(self, q_error, omega):
        axis = -quat.quat_to_axis(q_error) # determine axis of rotation
        omega_target = axis*self.bangbang_rate
        omega_error = omega_target - omega
        axis_torque = self.satInertia @ omega_error/self.updateTime # tau = I*omega/dt
        return axis_torque # negate to account for reaction wheel opposite reactions
    
    def sliding_bangbang_quat(self, q_error, omega, currentTimeNanos):            
        error_angle_degrees = quat.error_angle(q_error) # get minimum error angle (in degrees)
        self.mode_check(error_angle_degrees, currentTimeNanos) # switch modes based on current error

        if self.mode == "precise":
            return self.quaternion_controller(q_error, omega)
        elif self.mode == "slew":
            return self.bang_bang_controller(q_error, omega)
        else:
            print("MANUAL ERROR: Undefined controller mode", flush = True)