import numpy as np
from adcs_lib import quaternion, vector
from time import time

class MayhewTrigger():
    '''
    The use of the sign function in a controller can lead to the "chattering" phenomenon when noise is injected into state estimates.
    This is one way to implement an alternative to sign() with hysteretic behavior, in order to trade some efficiency for robustness.
    We lose the guarantee of taking the shortest angular path for attitude errors close to 180 deg, but it's worth it.
    For the mathematical details of a hybrid control system, refer to Mayhew, Sanfelice, Teel 2011 "Quaternion-Based Hybrid Control for Robust Global Attitude Tracking".

    Parameters
    ----------
    x : float
        Initial value of whichever variable whose sign we will be monitoring.
    half_width : float
        Hysteresis half-width, between 0 and 1. The rigorous way of determining this given sensor error is in that paper.
        If 0, this becomes a simple sign function. If >= 1, this becomes a constant.
    '''
    def __init__(self, x, half_width):
        sign = np.sign(x)
        self.memory_state = sign if sign != 0 else 1
        self.half_width  = half_width

    def step(self, x):
        '''
        Discrete dynamics for the memory variable so that sign only changes when variable pushes past half-width.
        Handles case of half-width = 0 by making sure memory variable retains value rather than being set to 0.

        Parameters
        ----------
        x : float
            Present value of variable.
        '''
        if x * self.memory_state <= - self.half_width:
            sign = np.sign(x)
            self.memory_state = sign if sign != 0 else self.memory_state

    def sign(self, x):
        '''
        Replicates the functionality of the sign function but with hysteresis near 0.

        Parameters
        ----------
        x : float
            Variable whose sign we are extracting.

        Returns
        -------
        int
            Either 1 or -1, depending on present value of variable and its recent history.
        '''
        self.step(x)
        return self.memory_state


class MagnetorquerController:
    '''
    Class for calculating magnetorquer controls and commands.

    Parameters
    ----------
    model : dynamic.ReducedDynamicalSystem
        Complete satellite, dynamics, and environmental model from inside Kalman filters.
    bang_bang : bool
        True if controller is bang-bang, false if we can modulate actuators.
    '''
    def __init__(self, model, bang_bang):
        # approximate quasi-optimal gain for detumbling, see page 310 formula
        self.detumble_gain      = 1.72192143 * 5.49370596e-3 * 4 * np.pi / 5560.8
        self.bbq                = np.array([0, 0, 0.00733]) # per thermal desktop
        self.converged          = False
        self.aligned            = True
        self.MoI                = model.satellite.reduced_moment
        self.clock              = model.clock
        eps                     = np.radians(23.439291)
        self.ecl                = np.array([np.cos(0.5 * eps), np.sin(0.5 * eps), 0, 0])

        # gains should be arguments, do later
        delta                   = 0.000#1 # this should almost certainly just be 0
        w_n                     = 0.028 #0.05
        zeta                    = 0.707
        self.dt                 = 0.05
        self.k_p                = (2 * w_n**2 + delta * w_n * zeta * 4) * self.MoI
        self.k_d                = (w_n* zeta * 2 + delta) * self.MoI
        self.k_i                = 2 * delta * w_n**2 * self.MoI

        #print(self.detumble_gain, np.diag(self.k_p), np.diag(self.k_d), np.diag(self.k_i), '\n')
        self.bang_bang          = bang_bang
        self.torquers           = model.satellite.magnetorquers
        self.error_trigger      = MayhewTrigger(1, 0.3)

        self.q_e_i              = np.zeros(3)

        if bang_bang:
            self.triggers       = [MayhewTrigger(1, 0.3) for i in range(3)]

    def actuator_commands(self, state, torque_command):
        '''
        Distributes commanded torque and calculates currents to send to actuators.

        Parameters
        ----------
        state : numpy.ndarray
            Array full of arrays for state variables and measurements.
        torque_command : numpy.ndarray
            Commanded torque for subsystem to provide.

        Returns
        -------
        numpy.ndarray
            Current (A) for x, y, z magnetorquers.
        '''
        mag_field                = state[5]
        magnetorquer_command     = np.cross(mag_field, torque_command) / np.dot(mag_field, mag_field) # magnetic moment

        if self.bang_bang:
            magnetorquer_command = np.array([trigger.sign(magnetorquer_command[i]) * self.torquers.torquers[2].max_m
                                             for i, trigger in enumerate(self.triggers)])

        current_commands         = self.torquers.distribute(magnetorquer_command)
        return current_commands

    def linear_error_tracking(self, state, cmd_data):
        '''
        Generalized tracking control with linear error dynamics. Reduces to a regulator when rate_cmd = 0.
        Inspired by Paelli and Bach 1993 "Attitude Control with Realization of Linear Error Dynamics", but differs in some details.
        Work in progress still.

        Parameters
        ----------
        state : numpy.ndarray
            Array full of arrays for state variables and measurements in their usual ordering.
        cmd_data : numpy.ndarray
            First slot for rate commands, second for attitude commands.

        Returns
        -------
        numpy.ndarray
            Ideal torque in order to track reference input.
        '''
        position, velocity, attitude, angular_vel = state[:4]
        rate_cmd           = cmd_data[0]
        q_cmd              = cmd_data[1]

        attitude_error     = quaternion.error_quat(attitude, q_cmd)
        self.q_e_i        += attitude_error[1:] * self.dt
        rate_error         = angular_vel - quaternion.sandwich(attitude_error, rate_cmd)

        Q_inv              = np.linalg.inv((vector.cross(attitude_error[1:]) + attitude_error[0]*np.eye(3)))
        q_term             = - 0.5 * (self.error_trigger.sign(attitude_error[0]) * 4 * 0.05**2  - np.dot(rate_error, rate_error)) * Q_inv.dot(attitude_error[1:])
        w_term             = - 0.707 * 2 * 0.05 * rate_error
        v                  = self.MoI.dot(q_term + w_term - 0.0000000 * Q_inv.dot(self.q_e_i))

        gyro_term          = np.cross(angular_vel, self.MoI.dot(angular_vel)) # this should be correct gyroscopic term

        control_law        = v + gyro_term

        tracking_rate      = np.linalg.norm(rate_error) < 0.0015
        tracking_attitude  = 1 - abs(attitude_error[0]) < 0.005

        self.converged     = tracking_attitude and tracking_rate

        return control_law

    def tracking(self, state, cmd_data):
        '''
        Generalized tracking control. Reduces to a regulator when rate_cmd = 0.
        For the mathematical details of a hybrid control system, refer to Mayhew, Sanfelice, Teel 2011 "Quaternion-Based Hybrid Control for Robust Global Attitude Tracking".

        Parameters
        ----------
        state : numpy.ndarray
            Array full of arrays for state variables and measurements in their usual ordering.
        cmd_data : numpy.ndarray
            First slot for rate commands, second for attitude commands.

        Returns
        -------
        numpy.ndarray
            Ideal torque in order to track reference input.
        '''
        position, velocity, attitude, angular_vel = state[:4]
        rate_cmd           = cmd_data[0]
        q_cmd              = cmd_data[1]

        attitude_error     = quaternion.error_quat(attitude, q_cmd)
        self.q_e_i        += attitude_error[1:] * self.dt
        rate_error         = angular_vel - quaternion.sandwich(attitude_error, rate_cmd)

        quat_term          = - self.error_trigger.sign(attitude_error[0]) * self.k_p.dot(attitude_error[1:])
        rate_term          = - self.k_d.dot(rate_error)

        gyro_term          = np.cross(angular_vel, self.MoI.dot(rate_error)) # this should be correct gyroscopic term
        #gyro_term          = np.cross(angular_vel, self.MoI.dot(angular_vel)) # this should be an acceptable gyroscopic term

        integ_term         = - self.k_i.dot(self.q_e_i)

        control_law        = gyro_term + quat_term + rate_term + integ_term

        tracking_rate      = np.linalg.norm(rate_error) < 0.0015
        tracking_attitude  = 1 - abs(attitude_error[0]) < 0.005

        self.converged     = tracking_attitude and tracking_rate

        return control_law

    def detumble(self, state):
        '''
        Global asymptotically stable control law for detumbling a satellite.
        Refer to "Magnetic Detumbling of a Rigid Spacecraft" (Avanzini and Giulietti, 2012).
        This is a generalization of B-Dot control, and preferred when gyro measurements are available.


        Parameters
        ----------
        mag_field : numpy.ndarray
            3D array of floats for body frame magnetic field data.
        gyro : numpy.ndarray
            3D array of floats for body frame gyroscope data.

        Returns
        -------
        numpy.ndarray
            Current (A) for x, y, z magnetorquers to induce required magnetic moment that detumbles satellite.
        '''

        gyro             = state[3]
        torque_command   = - self.detumble_gain * gyro
        currents_command = self.actuator_commands(state, torque_command)

        return currents_command


    def ecliptic_attitude(self, roll, q):
        '''Attitude for BBQ roll manuever, which puts satellite into its standby mode.
        Rotates at constant rate in plane of ecliptic around satellite z axis.

        Parameters
        ----------
        roll : bool
            True if time-varying rotation. False if static attitude (perhaps for just lining up with pole).

        Returns
        -------
        numpy.ndarray
            Unit quaternion encoding desired attitude.
        '''
        if roll:
            t      = self.clock.absolute
            rot    = np.array([np.cos(0.5 * (self.bbq[2] * t)), 0, 0, np.sin(0.5 * (self.bbq[2] * t))])
            q_d    = quaternion.product(self.ecl, rot)
        else:
            q_d    = self.ecl
        return q_d

    def induce_bbq_roll(self, state, roll):
        '''
        Maneuver satellite into spin-stabilized trajectory orthogonal to the sun for safe mode.

        Parameters
        ----------
        state : numpy.ndarray
            Array full of arrays for state variables and measurements in their usual ordering.

        Returns
        -------
        numpy.ndarray
            Current (A) for x, y, z magnetorquers to induce required magnetic moment in order to rotate around z-axis in ecliptic plane.
        '''
        q_cmd            = self.ecliptic_attitude(roll, state[2])
        w_cmd            = self.bbq if roll else np.zeros(3)
        bbq_setup        = self.tracking(state, [w_cmd, q_cmd])
        currents_command = self.actuator_commands(state, bbq_setup)

        return currents_command


class ReactionWheelsController:
    '''
    Class for calculating controls and acceleration commands for reaction wheels.

    Parameters
    ----------
    model : dynamic.ReducedDynamicalSystem
        Complete satellite, dynamics, and environmental model from inside Kalman filters.
    damping : float
        Damping ratio. Traditionally set to 0.707, but 1.0 is another option for critical damping.
    nat_freq : float
        Natural frequency. Haven't tuned this at all yet.
    '''
    def __init__(self, model, damping, nat_freq):
        self.satellite         = model.satellite
        self.MoI               = model.satellite.reduced_moment
        self.reaction_wheels   = model.satellite.reaction_wheels
        self.sun_vector        = lambda x: model.enviro.sun_vector(model.clock, x)
        self.error_trigger     = MayhewTrigger(1, 0.3)

        # reference Quaternion feedback regulator, Wie et al
        # These are optimal (in some sense) gains, but it's probably simplest to use classical methods here.
        #msum = np.sum(np.diag(self.MoI))
        #minvsum = sum([1/Ji for Ji in np.diag(self.MoI)])
        #msqrsum = sum([Ji**2 for Ji in np.diag(self.MoI)])
        #alpha = (9 - minvsum*msum) / (3*msqrsum - msum**2)
        #beta = (minvsum*msqrsum - 3*msum) / (3*msqrsum - msum**2)
        #self.quat_gain = np.linalg.inv(alpha*self.MoI + beta*np.identity(3))

        # these gains correspond to linearized system, should be good for < 90 deg maneuvers
        self.quat_gain = 2 * nat_freq**2 * self.MoI
        self.rate_gain = 2 * damping * nat_freq * self.MoI
        #print(self.quat_gain, self.rate_gain)

    def acquire_target(self, position, attitude, boresight, target):
        '''
        Calculates required rotation to align the boresight with a target.

        Parameters
        ----------
        position : numpy.ndarray
            3D array of floats for inertial frame position data.
        attitude : numpy.ndarray
            4D array of floats for inertial frame attitude quaternion data.
        target : numpy.ndarray
            3D array of floats for inertial frame target location data.
        boresight : numpy.ndarray
            3D array of floats for body frame camera boresight data.

        Returns
        -------
        numpy.ndarray
            4D array of floats for the active inertial frame rotation quaternion,
            which makes the boresight vector point at the target.
        '''

        direction    = vector.pointing_vector(position, target) # pointing vector in inertial frame
        boresight_I  = quaternion.sandwich_opp(attitude, boresight) # expression of camera direction in inertial frame
        r, theta     = vector.R_axis_angle(boresight_I, direction) # transform from inertial frame to desired orientation
        return quaternion.axisangle_to_quat(r, theta) # active rotation that takes cam to direction in fixed inertial frame

    def point_and_stare(self, state, mission_data):
        '''
        "Point and Stare" maneuver commands to be performed in inertial frame. Takes an inertial target location and the body-referenced boresight.
        Refer to "A Quaternion Feedback Regulator for Spacecraft Eigenaxis Rotations" (Wie, Weiss, and Arapostathis, 1989).
        At a later time, we will want to distinguish between actual wheel torques and effective wheel torques but for now this is fine.

        Parameters
        ----------
        state : numpy.ndarray
            Array full of arrays for state variables and measurements in their usual ordering.

        Returns
        -------
        numpy.ndarray
            Effective wheel acceleration commands to affect the required torque.
        '''

        position           = state[0]
        attitude           = state[2]
        boresight          = mission_data[1]
        target_point       = mission_data[2]
        w_ref              = mission_data[3]

        commanded_rotation = self.acquire_target(position, attitude, boresight, target_point)
        q_cmd              = quaternion.product(attitude, commanded_rotation)
        torque_command     = self.tracking(state, [w_ref, q_cmd])
        accl_cmds          = self.actuator_commands(torque_command)

        return accl_cmds

    def tracking(self, state, mission_data):
        '''
        Generalized tracking control. Reduces to a regulator when rate_cmd = 0.
        For the mathematical details of a hybrid control system, refer to Mayhew, Sanfelice, Teel 2011 "Quaternion-Based Hybrid Control for Robust Global Attitude Tracking".

        Parameters
        ----------
        state : numpy.ndarray
            Array full of arrays for state variables and measurements in their usual ordering.
        cmd_data : numpy.ndarray
            First slot for rate commands, second for attitude commands.

        Returns
        -------
        numpy.ndarray
            Ideal torque in order to track reference input.
        '''
        position, velocity, attitude, angular_vel = state[:4]
        S_inertial         = self.sun_vector(position)
        rate_cmd           = mission_data[0]
        q_cmd              = mission_data[1]

        attitude_error     = quaternion.error_quat(attitude, q_cmd)
        rate_error         = angular_vel - quaternion.sandwich(attitude_error, rate_cmd)

        gyro_term          = np.cross(angular_vel, self.MoI.dot(rate_error)) # this should be correct gyroscopic term
        #gyro_term          = np.cross(angular_vel, self.MoI.dot(angular_vel)) # this should be an acceptable gyroscopic term
        quat_term          = - self.error_trigger.sign(attitude_error[0]) * self.quat_gain.dot(attitude_error[1:])
        #quat_term          = self.quat_gain.dot(self.allowable_zone(q_cmd, attitude, S_inertial))
        mult               = 1 #+ np.dot(q_cmd[1:], q_cmd[1:]) # this was for experimenting with non-linear controls
        rate_term          = - mult * self.rate_gain.dot(rate_error)

        control_law        = gyro_term + quat_term + rate_term

        return control_law

    def actuator_commands(self, control_law):
        '''
        Distributes torque and calculates acceleration commands for wheels.
        At a later time, we will want to distinguish between actual wheel torques and effective wheel torques but for now this is fine.

        Parameters
        ----------
        control_law : numpy.ndarray
            Ideal torque command.

        Returns
        -------
        numpy.ndarray
            Accelerations for each reaction wheel.
        '''
        acceleration_cmds = self.reaction_wheels.accelerations(- control_law)
        return acceleration_cmds

    def allowable_zone(self, q_cmd, q_act, obj):
        '''
        Experimental control law for avoiding pointing at the sun. Doesn't work yet.
        Following approach of Lee & Mesbahi 2014 "Feedback Control for Spacecraft Reorientation Under Attitude Constraints Via Convex Potentials".
        It's straightforward to generalize this to multiple objects, but afaik, we only have to wory about the sun.

        Parameters
        ----------
        q_cmd : numpy.ndarray
            Commanded attitude.
        q_act : numpy.ndarray
            Measured attitude.
        obj : numpy.ndarray
            Object to avoid pointing sensitive instruments at or require pointing near.

        Returns
        -------
        numpy.ndarray
            Attitude term of feedback control law, still requires an angular rate term.
        '''
        p           = np.dot(q_cmd, q_act)
        q_star      = quaternion.conjugate(q_act)
        scalar_term = 0
        vector_term = np.zeros(4)
        for instr in self.satellite.instruments:
            foo          = -1.0 if instr.forbidden else 1.0
            M            = instr.quadratic_form(obj)
            Mq           = M.dot(q_act)
            qMq          = q_act.T.dot(Mq)
            scalar_term += np.log(foo * 0.5 * qMq)
            vector_term += quaternion.product(q_star, Mq) / qMq

        first_term  = -0.5 * self.error_trigger.sign(p) * scalar_term * quaternion.product(q_star, q_cmd)[1:]
        second_term = (1 - np.abs(p)) * vector_term[1:]
        return first_term + second_term
