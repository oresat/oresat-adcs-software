import numpy as np
import quaternion

class MagnetorquerController:
    '''
    Class for calculating magnetorquer commands.
    At some point (low priority) we may want to consider changing output from a
    desired magnetic moment to the desired currents.
    '''

    def __init__(self):
        # approximate control gain, see page 310 formula. at some point do calculation for magnetic equator instead of actual equator
        self.gain = 1.784127 * 5.09404743e-3 * 4 * np.pi / 5560.8

    # bdot control for detumbling. we'll want something only slightly different for control in general
    # could also bang-bang this, see pg 309
    def detumble(self, mag_field, gyro):
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

        magnetorquer_command = -self.gain * np.cross(mag_field, gyro) / np.dot(mag_field, mag_field)
        return magnetorquer_command

class ReactionWheelsController:
    '''
    Class for calculating torque commands for reaction wheels to provide.
    At some point (med priority) we need to make sure the outputs are of a
    compatible type with the inputs to the plant.
    '''

    def __init__(self, satellite, damping, nat_freq):
        self.cancel_gyroscopic = False # set to True if you want to cancel gyroscopic torques. necessity depends on application.
        self.MoI = satellite.reduced_moment
        self.reaction_wheels = satellite.reaction_wheels
        #self.orbital_ref = np.array([0, -2* np.pi / 5560.8, 0]) # might not need this

        # reference Quaternion feedback regulator, Wie et alnumpy.ndarray
        # These are optimal (in some sense) gains, but it's probably simplest to use classical methods here.
        #msum = sum(BODY_MOMENTS)
        #minvsum = sum([1/Ji for Ji in BODY_MOMENTS])
        #msqrsum = sum([Ji**2 for Ji in BODY_MOMENTS])
        #alpha = (9 - minvsum*msum) / (3*msqrsum - msum**2)
        #beta = (minvsum*msqrsum - 3*msum) / (3*msqrsum - msum**2)
        #self.quat_gain = np.linalg.inv(alpha*MOMENT_OF_INERTIA + beta*np.identity(3))
        #self.rate_gain = 1.0 * MOMENT_OF_INERTIA # rate gain arbitrary right now, balanced with quat gain

        # these gains correspond to linearized system, should be good for < 90 deg maneuvers
        self.quat_gain = 2 * nat_freq**2 * self.MoI
        self.rate_gain = 2 * damping * nat_freq * self.MoI

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

    def point_and_stare(self, position, velocity, angular_vel, attitude, target_point, camera_boresight):
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

        commanded_rotation = self.acquire_target(position, attitude, target_point, camera_boresight)
        attitude_error = quaternion.error_quat(attitude, commanded_rotation)
        gyroscopic_term = np.zeros(3) if not self.cancel_gyroscopic else np.cross(angular_vel, self.MoI.dot(angular_vel))
        # sign is so that we take shortest path in S^3
        quat_term = np.sign(attitude[0]) * self.quat_gain.dot(attitude_error[1:])
        rate_term = self.rate_gain.dot(angular_vel)
        reaction_wheels_command = gyroscopic_term - quat_term - rate_term
        return self.reaction_wheels.accelerations(-reaction_wheels_command) # minus or plus???

    def bbq_roll(self):
        '''Placeholder for BBQ roll manuever, which puts satellite into its standby mode.'''
        return np.zeros(3)

    def cfc_acquire(self):
        '''Placeholder for CFC acquiring attitude'''
        return np.zeros(3)

    def cfc_hold(self):
        '''Placeholder for CFC holding inertial attitude'''
        return np.zeros(3)
