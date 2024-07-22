import numpy as np
from . import quaternion, vector
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
            print("jump!")
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
        self.bbq                = np.array([0, 0, 0.00733])*4 # per thermal desktop
        eps                     = np.radians(23.439291)
        self.ecl                = np.array([np.cos(0.5 * eps), np.sin(0.5 * eps), 0, 0])
        self.satellite          = model.satellite
        self.MoI                = model.satellite.total_moment
        self.g_torque           = lambda x, q: -model.enviro.grav_torque(x, q, self.MoI)
        self.clock              = model.clock
        self.hd_body  = self.MoI.dot(self.bbq)
        self.Hd_intl  = np.linalg.norm(self.hd_body) * quaternion.sandwich_opp(self.ecl, [0, 0, 1])

        self.k_bbq = 1.72192143 * 4 * np.pi / 5560.8
        self.detumble_gain      = self.MoI[0][0] * self.k_bbq
        print('xi:', self.k_bbq)

        #print(self.detumble_gain, np.diag(self.k_p), np.diag(self.k_d), np.diag(self.k_i), self.k_whl, '\n')
        self.bang_bang          = bang_bang
        self.torquers           = model.satellite.magnetorquers
        self.error_trigger      = MayhewTrigger(1, 0.3)

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

    def desaturate(self, state, wheel_ref):
        h_w = self.satellite.reaction_wheels.momentum(state[4])
        mag_field                = state[5]
        torque_command   = - self.k_bbq * (h_w - self.satellite.reaction_wheels.momentum(wheel_ref))
        current_commands = self.actuator_commands(state, torque_command)

        return current_commands

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
        #torque_command   = - self.k_bbq * self.MoI[0][0] * gyro # uniform gain
        torque_command   = - self.k_bbq * self.MoI.dot(gyro) # axis-sensitive gain
        currents_command = self.actuator_commands(state, torque_command)

        return currents_command

    def no_q_bbq_roll(self, state):
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
        h = self.MoI.dot(state[3])
        Hd = quaternion.sandwich(state[2], self.Hd_intl)
        purespin = h - self.hd_body
        spinaxis = h - Hd
        torque_command   = - self.k_bbq * (purespin + spinaxis)
        currents_command = self.actuator_commands(state, torque_command)

        return currents_command

    def check(self, q, w):
        h = self.MoI.dot(w)
        Hd = quaternion.sandwich(q, self.Hd_intl)
        purespin = h - self.hd_body
        spinaxis = h - Hd
        return np.linalg.norm(purespin + spinaxis)


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
        self.MoI               = model.satellite.total_moment
        self.reaction_wheels   = model.satellite.reaction_wheels
        self.clock             = model.clock
        self.obj_avoidance     = True
        self.objects           = [lambda state: quaternion.sandwich(state[2], model.enviro.sun_vector(model.clock, state[0])),
                                lambda state: -vector.normalize(state[0])]
        self.error_trigger     = MayhewTrigger(1, 0.3)
        self.bbq               = np.array([0, 0, 0.00733])*4 # per thermal desktop
        eps                    = np.radians(23.439291) # assuming const for now
        self.ecl               = np.array([np.cos(0.5 * eps), np.sin(0.5 * eps), 0, 0])
        self.g_torque          = lambda x, q: -model.enviro.grav_torque(x, q, self.MoI)

        # reference Quaternion feedback regulator, Wie et al
        # These are optimal (in some sense) gains, but it's probably simplest to use classical methods here.
        # note these have nothing to do with the alpha or beta below here, just ran out of letters
        #msum = np.sum(np.diag(self.MoI))
        #minvsum = sum([1/Ji for Ji in np.diag(self.MoI)])
        #msqrsum = sum([Ji**2 for Ji in np.diag(self.MoI)])
        #alpha = (9 - minvsum*msum) / (3*msqrsum - msum**2)
        #beta = (minvsum*msqrsum - 3*msum) / (3*msqrsum - msum**2)
        #self.quat_gain = np.linalg.inv(alpha*self.MoI + beta*np.identity(3))

        # these gains correspond to linearized system, should be good for < 90 deg maneuvers
        stop         = np.pi#0.5 * self.MoI[0][0] * 0.25**2 / 0.005 #2/5 * np.pi
        self.epsilon = 1#0.95#np.cos(stop) #stop * 3 / 5 #0.75
        print(stop, self.epsilon)
        self.alpha   = 2 * nat_freq**2
        for instr in self.satellite.instruments:
            instr.set_gains(stop, [self.alpha * self.epsilon for obj in self.objects])
            print('sigma:', instr.sigma)
        self.J     = self.MoI
        self.alpha *= 1#0.05
        #self.J     = np.eye(3) * np.amax(self.MoI)
        self.gamma = 2 * damping * nat_freq
        #self.beta  = 2 / damping * nat_freq * self.delta# * 5.49370596e-3
        print('alpha', self.alpha*self.J[0][0], 'beta', self.alpha*self.J[0][0] * self.epsilon, 'gamma', self.gamma*self.J[0][0])
        print("t_s:", 4/(damping * nat_freq), 'to', 8/(damping * nat_freq))

    def track_and_desaturate(self, state, magnetorque, mission_data):
        u = self.tracking(state, mission_data)
        mag_term = np.cross(state[5], magnetorque)
        torque = u + mag_term
        return self.actuator_commands(torque)

    def ecliptic_attitude(self, roll):
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

    def induce_bbq_roll_attitude(self, state, magnetorque):
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
        q_cmd            = self.ecliptic_attitude(True)
        w_cmd            = self.bbq
        bbq_setup        = self.track_and_desaturate(state, magnetorque, [w_cmd, np.zeros(3), q_cmd])
        return bbq_setup

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
        direction_B = quaternion.sandwich(attitude, direction)
        r, theta     = vector.R_axis_angle(boresight, direction_B) # transform from inertial frame to desired orientation
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

        h = position - target_point
        w_ref_intl = np.cross(h, state[1]) / np.dot(h, h) # match angular velocity to track passively, probably not worth taking derivative
        w_ref = quaternion.sandwich(attitude, w_ref_intl)
        #w_ref = np.array([np.linalg.norm(w_ref_intl), 0, 0])
        #w_ref = np.array([2*np.pi/5560, 0, 0]) # approx. match angular velocity to track passively
        #w_ref = np.zeros(3) # this is a safe alternative

        commanded_rotation = self.acquire_target(position, attitude, boresight, target_point)
        q_cmd              = quaternion.product(attitude, commanded_rotation)
        torque_command     = self.tracking(state, [w_ref, np.zeros(3), q_cmd])
        accl_cmds          = self.actuator_commands(torque_command)

        return accl_cmds

    def tracking(self, state, desired_state):
        '''
        Generalized tracking control. Reduces to a regulator when rate_cmd = 0.
        For the mathematical details of a hybrid control system, refer to Mayhew, Sanfelice, Teel 2011 "Quaternion-Based Hybrid Control for Robust Global Attitude Tracking".

        Parameters
        ----------
        state : numpy.ndarray
            Array full of arrays for state variables and measurements in their usual ordering.
        desired_state : numpy.ndarray
            First slot for rate commands, second for derivative of rate commands, third for attitude commands.

        Returns
        -------
        numpy.ndarray
            Ideal torque in order to track reference input.
        '''
        position, velocity, attitude, angular_vel, wheel_vel = state[:5]
        objects            = [obj(state) for obj in self.objects]
        rate_cmd           = desired_state[0]
        rate_cmd_dot       = desired_state[1]
        q_cmd              = desired_state[2]

        h_w                = self.satellite.reaction_wheels.momentum(wheel_vel)
        attitude_error     = quaternion.error_quat(attitude, q_cmd)
        rate_cmd_B         = quaternion.sandwich(attitude_error, rate_cmd)
        rate_error         = angular_vel - rate_cmd_B
        del_H              = self.MoI.dot(rate_error)

        gyro_term          = np.cross(angular_vel, self.MoI.dot(angular_vel) + h_w) # this should be correct gyroscopic term
        #gyro_term          = np.cross(angular_vel, self.MoI.dot(angular_vel)) # this should be an acceptable gyroscopic term
        #gyro_term          = np.zeros(3) # we don't really need the gyroscopic term

        feedforward_terms  = - self.MoI.dot(np.cross(angular_vel, rate_cmd_B) + quaternion.sandwich(attitude_error, rate_cmd_dot))

        if self.obj_avoidance:
            quat_term          = self.J.dot(self.allowable_zone(q_cmd, attitude, attitude_error, rate_error, objects))
        else:
            quat_term          = -self.error_trigger.sign(attitude_error[0]) * self.alpha * self.J.dot(attitude_error[1:])

        mult               = 1 # we don't need even more non-linear controls
        #mult               = np.dot(attitude_error[1:], attitude_error[1:]) # this was for experimenting with non-linear controls
        #mult               = 2 + attitude_error[0]**2 # this was for experimenting with non-linear controls
        rate_term          = - mult * self.gamma * del_H

        control_law        = gyro_term + quat_term + rate_term + feedforward_terms

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

    def allowable_zone(self, q_cmd, q_act, del_q, del_omega, objects):
        '''
        Experimental control law for avoiding pointing at the sun. Seems to work ok but needs proof.
        Following approach of Lee & Mesbahi 2014 "Feedback Control for Spacecraft Reorientation Under Attitude Constraints Via Convex Potentials".
        It's straightforward to generalize this to multiple objects, but afaik, we only have to wory about the sun.
        We could calculate quadratic form(s) pretty infrequently if we wanted to.

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
        s               = self.error_trigger.sign(del_q[0])
        c               = self.alpha
        constraint_axis = np.zeros(3)
        nudge_term      = np.zeros(3)

        for instr in self.satellite.instruments:
            for j, obj in enumerate(objects):
                psi = 0.5 * instr.sign[j] * (np.dot(instr.boresight, obj) - instr.cos_psi[j])
                if instr.cares_about(j, psi):
                    if psi <= 0: # a kludge, just in case we feed it garbage
                        print('attitude constraint violated:',psi)
                        psi = 1e-4
                    #t = np.min([1, psi/instr.sigma[j]])
                    #t = 3 * (psi/instr.sigma[j])**2 - 2 * (psi/instr.sigma[j])**3
                    #c *= t
                    c               -= instr.gains[j] * np.log(psi)
                    #print(self.alpha, -instr.gains[j] * np.log(psi))
                    #print(t,self.alpha*t, -instr.gains[j] * np.log(psi))
                    perp             = np.cross(instr.boresight, obj)
                    constraint_axis += instr.gains[j] * instr.sign[j] / psi * perp
                    if psi < instr.iota[j] and del_q[1:] is not np.zeros(3):
                        z = s * np.cross(del_q[1:], perp)
                        if z is np.zeros(3):
                            z = s * np.cross(del_q[1:], np.random.uniform(size=3))
                        z   = vector.normalize(z)
                        dir = np.sign(np.dot(del_omega, z))
                        dir = dir if dir != 0 else instr.sign[j]
                        nudge_term -= instr.push[j] * dir * z

        attractive_term  = -s * c * del_q[1:]
        repulsive_term   = 0.5 * (1 - s * del_q[0]) * constraint_axis
        #print(c, 0.5 * (1 - s * del_q[0]))
        #print(del_q[1:], constraint_axis)
        #print(np.linalg.norm(attractive_term ), np.linalg.norm(repulsive_term) , np.linalg.norm(nudge_term))
        return attractive_term + repulsive_term + nudge_term
