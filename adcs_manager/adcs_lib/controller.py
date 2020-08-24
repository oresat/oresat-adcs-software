import numpy as np
from adcs_lib import quaternion

class MayhewTrigger():
    '''
    The use of the sign function in a controller can lead to the "chattering" phenomenon when noise is injected into state estimates.
    This is one way to implement an alternative to sign() with hysteretic behavior, in order to trade some efficiency for robustness.
    We lose the guarantee of taking the shortest angular path for attitude errors close to 180 deg, but it's worth it.
    For the mathematical details of a hybrid control system, refer to Mayhew, Sanfelice, Teel 2011.

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
            print("sign switch!") # for debugging

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
    Class for calculating magnetorquer commands.
    At some point (low priority) we may want to consider changing output from a
    desired magnetic moment to the desired currents.
    '''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self, torquers, bang_bang):
        # approximate control gain, see page 310 formula
        self.gain      = 1.72192143 * 5.09404743e-3 * 4 * np.pi / 5560.8
        print(self.gain)
        self.bang_bang = bang_bang
        self.torquers  = torquers

        self.saturates = True
        self.max_moment = 1.243 # A m^2
        if bang_bang:
            self.triggers = [MayhewTrigger(1, 0.3) for i in range(3)]

    def actuator_commands(self, state, torque_command):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        mag_field = state[5]
        B_mat = quaternion.cross(mag_field)
        #magnetorquer_command = np.linalg.inv(B_mat.dot(B_mat.T) + np.eye(3)*self.gain**2).dot(B_mat.dot(torque_command))
        magnetorquer_command = np.cross(mag_field, torque_command) / np.dot(mag_field, mag_field)
        #print(magnetorquer_command, magnetorquer_command2)
        if self.bang_bang:
            magnetorquer_command = np.array([trigger.sign(magnetorquer_command[i]) * self.torquers.torquers[2].max_m
                                             for i, trigger in enumerate(self.triggers)])
        current_commands = self.torquers.distribute(magnetorquer_command)

        return current_commands

    # bdot control for detumbling. we'll want something only slightly different for control in general
    # could also bang-bang this, see pg 309
    def detumble(self, state):
        '''
        Global asymptotically stable control law for detumbling a satellite.
        Refer to "Magnetic Detumbling of a Rigid Spacecraft" (Avanzini and Giulietti, 2012).


        Parameters
        ----------
        mag_field : numpy.ndarray
            3D array of floats for body frame magnetic field data.
        gyro : numpy.ndarray
            3D array of floats for body frame gyroscope data.

        Returns
        -------
        numpy.ndarray
            3D array of floats for required body frame magnetic moment to detumble satellite.
        '''

        gyro      = state[3]
        torque_command = - self.gain * gyro
        currents_command = self.actuator_commands(state, torque_command)


        return currents_command

class ReactionWheelsController:
    '''
    Class for calculating torque commands for reaction wheels to provide.
    At some point (med priority) we need to make sure the outputs are of a
    compatible type with the inputs to the plant.
    '''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self, model, damping, nat_freq):
        self.cancel_gyroscopic = False # set to True if you want to cancel gyroscopic torques. necessity depends on application.
        self.sun_exclusion     = False
        self.converged         = False
        self.w_b               = 0.00733 # per thermal desktop
        self.bbq               = np.array([0, 0, self.w_b])
        self.satellite         = model.satellite
        self.MoI               = model.satellite.reduced_moment
        self.reaction_wheels   = model.satellite.reaction_wheels
        self.clock             = model.clock
        self.enviro            = model.enviro
        self.error_trigger     = MayhewTrigger(1, 0.3)

        #self.orbital_ref = np.array([0, -2* np.pi / 5560.8, 0]) # might not need this

        # reference Quaternion feedback regulator, Wie et alnumpy.ndarray
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
        self.wheel_gain = self.rate_gain[0][0] * 0.75
        print(np.linalg.norm(self.quat_gain), np.linalg.norm(self.rate_gain))

    def acquire_target(self, position, attitude, target, boresight):
        '''
        Calculates required maneuver to align the boresight with a target.

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

        def pointing_vector(position, target):
            '''
            Unit pointing vector.

            Parameters
            ----------
            position : numpy.ndarray
                3D array of floats for inertial frame position data.
            target : numpy.ndarray
                3D array of floats for inertial frame target location data.

            Returns
            -------
            numpy.ndarray
                A unit vector which points from the satellite's position to a target location.
            '''

            p = target - position
            return quaternion.normalize(p)

        def R_axis_angle(boresight, point):
            '''
            Rotate one vector into another.

            Parameters
            ----------
            boresight : numpy.ndarray
                3D array of floats for inertial frame camera boresight data.
            point : numpy.ndarray
                3D array of floats for inertial frame pointing vector data.

            Returns
            -------
            numpy.ndarray
                3D array for inertial frame axis of rotation.
            float
                Angle of rotation, with usual right-handed convention.
            '''
            r     = quaternion.normalize(np.cross(boresight, point))
            # clip is to keep floating pt error from crashing arccos
            theta = np.arccos(np.clip(np.dot(boresight, point), -1, 1))
            return r, theta # use + for active (- for passive)

        direction    = pointing_vector(position, target) # pointing vector in inertial frame
        boresight_I  = quaternion.sandwich(quaternion.conjugate(attitude), boresight) # expression of camera direction in inertial frame
        r, theta     = R_axis_angle(boresight_I, direction) # transform from inertial frame to desired orientation
        return quaternion.axisangle_to_quat(r, theta) # active rotation that takes cam to direction in fixed inertial frame

    def point_and_stare(self, state, mission_data):
        '''
        "Point and Stare" maneuver commands to be performed in inertial frame.
        Refer to "A Quaternion Feedback Regulator for Spacecraft Eigenaxis Rotations" (Wie, Weiss, and Arapostathis, 1989).

        Parameters
        ----------

        Takes an inertial target location and the body-referenced boresight.
        Returns the ideal torque vector for pointing a camera at target and tracking'''
        # I am not sure if we need or care about matching angular vel to orbit. Can think about that later.
        #LVLH_to_GCI = frame.lvlh_to_inertial(position, velocity) # debatably, we could be calculating this as we propagate state but seems wasteful
        # we want to rotate in both body and local orbital frame to match rotation around earth
        #rate_ref = (self.orbital_ref + sandwich(attitude, LVLH_to_GCI.dot(self.orbital_ref))) / 2

        position           = state[0]
        attitude           = state[2]
        target_point       = mission_data[1]
        boresight          = mission_data[2]
        commanded_rotation = self.acquire_target(position, attitude, target_point, boresight)
        return commanded_rotation


    def tracking(self, state, mission_data):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        position, velocity, attitude, angular_vel = state[:4]
        S_body             = state[6]
        S_inertial         = quaternion.sandwich_opp(attitude, S_body)
        rate_ref           = mission_data[0]
        q_cmd              = mission_data[1]
        gyroscopic_term    = np.zeros(3) if not self.cancel_gyroscopic else np.cross(angular_vel, self.MoI.dot(angular_vel))
        quat_term          = self.q_term(attitude, q_cmd, S_inertial)
        mult = 1 #+ np.dot(q_cmd[1:], q_cmd[1:])
        rate_term          = - self.rate_gain.dot(angular_vel - rate_ref) * mult
        control_law        = gyroscopic_term + quat_term + rate_term
        return control_law

    def transform_law_to_wheels(self, control_law):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return self.reaction_wheels.accelerations(-control_law)

    def ecliptic_attitude(self, roll):
        '''Placeholder for BBQ roll manuever, which puts satellite into its standby mode.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        eps    = np.radians(23.439291)
        t      = 0 if not roll else self.clock.absolute
        q_d    = np.array([np.cos(0.5 * (eps + self.w_b * t)), 0, 0, np.sin(0.5 * (eps + self.w_b * t))])
        return q_d

    def induce_bbq_roll(self, state):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        q = state[2]
        w = state[3]
        q_cmd = self.ecliptic_attitude(True)
        error_quat = quaternion.error_quat(q, q_cmd)
        w_cmd = quaternion.sandwich(error_quat, self.bbq)
        slide = np.cross(w, self.MoI.dot(w - w_cmd)) # might be faster, but might have larger steady state amplitudes
        #slide = quaternion.cross(w_cmd).dot(self.MoI.dot(w_cmd)) # this tried switching a few items

        tracking_rotation = np.linalg.norm(w[:2]) < 6.4e-4 and np.abs(w[2] - w_cmd[2]) < 1.5e-3
        colinear = np.linalg.norm(error_quat[1:3]/2) < 0.05
        self.converged = colinear and tracking_rotation

        bbq_setup = self.tracking(state, [w_cmd, q_cmd])
        return bbq_setup + slide

    def cfc_acquire(self):
        '''Placeholder for CFC acquiring attitude'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return np.zeros(3)

    def cfc_hold(self):
        '''Placeholder for CFC holding inertial attitude'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return np.zeros(3)

    def allowable_zone(self, q_cmd, q_act, obj):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        p           = np.dot(q_cmd, q_act)
        q_star      = quaternion.conjugate(q_act)
        scalar_term = 0
        vector_term = np.zeros(4)
        for instr in self.satellite.instruments:
            foo          = -1 if instr.forbidden else 1
            M            = instr.quadratic_form(obj)
            Mq           = M.dot(q_act)
            qMq          = q_act.T.dot(Mq)
            scalar_term += np.log(foo * 0.5 * qMq)
            vector_term += quaternion.product(q_star, Mq) / qMq

        first_term  = -0.5 * self.error_trigger.sign(p) * scalar_term * quaternion.product(q_star, q_cmd)[1:]
        second_term = (1 - np.abs(p)) * vector_term[1:]
        return first_term + second_term

    def q_term(self, attitude, commanded_rotation, S_inertial):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        if self.sun_exclusion:
            attitude_error = self.allowable_zone(commanded_rotation, attitude, S_inertial)
            quat_term      = self.quat_gain.dot(attitude_error)
        else:
            # sign is so that we take shortest path in S^3
            attitude_error = quaternion.error_quat(attitude, commanded_rotation)
            quat_term      = - self.error_trigger.sign(attitude_error[0]) * self.quat_gain.dot(attitude_error[1:])
        #print(self.quat_gain.dot(self.allowable_zone(commanded_rotation, attitude, S_inertial)),
        #        - self.error_trigger.sign(attitude_error[0]) * self.quat_gain.dot(quaternion.error_quat(attitude, commanded_rotation)[1:]))
        return quat_term

    def exit_mission_mode(self, state):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        w, W    = state[3], state[4]
        q = state[2]
        print(np.linalg.norm(W))
        bbq_setup = self.induce_bbq_roll(state)

        if np.linalg.norm(w - self.bbq) > 0.0025:
            return bbq_setup

        wheel_V   = - np.dot(W, W) * self.wheel_gain * self.reaction_wheels.wheels[0].par_mom_scalar
        sat_V   = np.dot(bbq_setup, w - self.bbq)
        #print(wheel_V, sat_V)
        momentum_manager = w - self.reaction_wheels.distribution.T.dot(W)
        M_norm_squared = np.dot(momentum_manager, momentum_manager)
        if M_norm_squared == 0:
            print('bad control!')
            return bbq_setup
        else:
            return momentum_manager / M_norm_squared * (wheel_V + sat_V)
