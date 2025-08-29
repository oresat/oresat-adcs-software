from Basilisk.architecture import sysModel, messaging
from Basilisk.utilities import macros
from array import array # C-style arrays for defining motor torques
import numpy as np
from ADCS_Discrete_State_Space_Calculator import get_gain_matrix
import Quaternions as quat # quaternion operations
from sys import exit

class FlightSoftware(sysModel.SysModel):
    def __init__(self, G_matrix, update_time, rw_Inertia, satInertia):
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
        self.maxSpeed = 10000 * macros.RPM # [rad]
        self.output_states = False # output state messages or not for debugging
        self.controllerStartTime = 0 # time at which controller should begin taking control [seconds]
        
        use_integrator = False # use gain matrix with integrator or without
        self.fast_gain = get_gain_matrix(satInertia, update_time, .1, 0.05, use_integrator)
        self.slow_gain = get_gain_matrix(satInertia, update_time, .05, 0.001, use_integrator)
        self.K = self.fast_gain
        self.mode = "slew"
        
        self.error = [] # used for tracking and graphing error
            
        print(f"Maximum reaction wheel speed {self.maxSpeed} rad/s")
        print(f"Maximum reaction wheel torque {self.maxTorque} Nm")
        print(f"Gain matrix K:\n{self.K}")
        
        print(f"Targeting angle change of ")

        self.target_num = 1
        self.val_array = []
        self.array_counter = 0
        self.sign_flipped = 0

    def Reset(self, currentTimeNanos):
        print(f"({self.ModelTag}) Reset called at {currentTimeNanos * macros.NANO2SEC:.2f} s")
        
    def UpdateState(self, currentTimeNanos):
        
        # gather system states
        if self.starTrackerMsgIn.isWritten():
            self.starTrackerMsg = self.starTrackerMsgIn()
            q = self.starTrackerMsg.qInrtl2Case  # [qs, q1, q2, q3]
            q = quat.to_scalar_last(q) # convert Basilisk quaternion to scalar last: [q1, q2, q3, qs]
            q = quat.normalize(q) # normalize
                        
        if self.imuMsgIn.isWritten():
            self.imuMsg = self.imuMsgIn()
            omega = self.imuMsg.AngVelPlatform
            omega_rpm = np.asarray(omega) * 60 / (2 * np.pi)
        
        if self.rwSpeedMsgIn.isWritten():
            self.rwSpeedMsg = self.rwSpeedMsgIn()
            wheelSpeeds = self.rwSpeedMsg.wheelSpeeds
        
        q_error = quat.quat_error(self.q_target, q) # get error quaternion, this function automatically sanitizes by performing normalization and hemisphere checks
        q_error = quat.hemi(q_error)
        self.error.append(q_error) # required for plotting after conclusion of sim
        
        if (currentTimeNanos * macros.NANO2SEC >= self.controllerStartTime):
            desired_torque = self.quaternion_controller(q_error, omega) # compute desired 3-axis torque from controller
            wheel_torque = self.convert_torque_to_wheels(desired_torque) # convert desired 3-axis torque to inputs for 4 wheels
            self.command_wheel_torques(currentTimeNanos, wheel_torque, wheelSpeeds) # Write the payload
            
    def command_wheel_torques(self, currentTimeNanos, wheel_torque, wheelSpeeds): # send commanded torque values to reaction wheels
        self.check_torque_vals(wheel_torque, wheelSpeeds) # ensure none of the torque values exceed max torque or accelerate wheel past max RPM in either direction and write to self.torque_vals
        self.rwMotorTorquePayload.motorTorque = self.torque_vals
        self.rwMotorTorqueOutMsg.write(self.rwMotorTorquePayload, currentTimeNanos, self.moduleID)    
          
    def convert_torque_to_wheels(self, torque_array): # convert 3-axis torque request to üyramid configuration reaction wheel output
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
    
    def zero_wheel_speeds(self, rwSpeeds, currentTimeNanos): # zero out all reaction wheel speeds
        for i, rwSpeed in enumerate(rwSpeeds[:4]):
            required_torque = -self.rwInertia*rwSpeed/self.updateTime
            self.torque_vals[i] = max(-self.maxTorque, min(required_torque, self.maxTorque))
        self.rwMotorTorquePayload.motorTorque = self.torque_vals
        self.rwMotorTorqueOutMsg.write(self.rwMotorTorquePayload, currentTimeNanos, self.moduleID)
    
    def quaternion_controller(self, q_error, omega):
        omega_error = omega - self.omega_target
        x = np.concatenate((q_error[:3], omega_error)) # assemble state vector
        return -self.K @ x # invert sign for control
