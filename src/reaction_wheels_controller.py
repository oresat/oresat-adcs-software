import numpy as np
from geometry import *
from system_definition import *



class ReactionWheelsController:
    def __init__(self):
        self.orbital_ref = np.array([0, -2* np.pi / 5560.8, 0])
        # reference Quaternion feedback regulator, Wie et al
        # there are other possible choices for gains we'll have to discuss
        # the other best method is classical, but gets both gains
        msum = sum(BODY_MOMENTS)
        minvsum = sum([1/Ji for Ji in BODY_MOMENTS])
        msqrsum = sum([Ji**2 for Ji in BODY_MOMENTS])
        alpha = (9 - minvsum*msum) / (3*msqrsum - msum**2)
        beta = (minvsum*msqrsum - 3*msum) / (3*msqrsum - msum**2)
        self.quat_gain = np.linalg.inv(alpha*MOMENT_OF_INERTIA + beta*np.identity(3))
        self.rate_gain = 1.0 * MOMENT_OF_INERTIA # rate gain arbitrary right now, balanced with quat gain
        #print(self.quat_gain, self.rate_gain)


    def detumble(self, adcs_data):

        # Do stuff and make command

        reaction_wheels_command = np.array((0.0 , 0.0, 0.0))
        return reaction_wheels_command

    # Point and Stare Maneuver performed in inertial frame (because attitude is inertial)
    # returns the ideal torque vector for pointing a camera at target and tracking
    def point(self, adcs_data, target_point, camera_boresight):
        def acquire_target(target, position, attitude, camera_boresight):
            # this unit vector points from the sat to an arbitrary target
            def pointing_vector(target, r):
                p = target - r
                return normalize(p)

            # returns parameters for an active rotation of camera vector
            # towards pointing vector with fixed inertial frame
            def R_axis_angle(camera, point):
                r = normalize(np.cross(camera, point))
                # clip is to keep floating pt error from crashing arccos
                theta = np.arccos(np.clip(np.dot(camera, point), -1, 1))
                return r, theta # use + for active (- for passive)

            direction = pointing_vector(target, position) # pointing vector in inertial frame
            camera_I = sandwich(conjugate(attitude), camera_boresight) # expression of camera direction in inertial frame
            r, theta = R_axis_angle(camera_I, direction) # transform from inertial frame to desired orientation
            #print(theta, "radians")
            return axisangle_to_quat(r, theta) # active rotation that takes cam to direction in fixed inertial frame

        position = np.array(adcs_data[0][0])
        velocity = np.array(adcs_data[0][1])
        angular_rate = np.array(adcs_data[2][0])
        attitude = np.array(adcs_data[1][0])
        #ra, dec, roll, _ = adcs_data[1]
        #attitude = eulerangle_to_quat(ra, dec, roll)
        LVLH_to_GCI = lvlh_to_inertial(position, velocity)
        # we want to rotate in both body and local orbital frame to match rotation around earth
        rate_ref = (self.orbital_ref + sandwich(attitude, LVLH_to_GCI.dot(self.orbital_ref))) / 2
        #print(sandwich(attitude, LVLH_to_GCI.dot(self.orbital_ref)))
        attitude_error = error_quat(attitude,
                                    acquire_target(target_point, position, attitude, camera_boresight))
        mu = 0 # 0 and 1 are the options, we might not need to cancel gyroscopic coupling
        gyroscopic_term = mu * np.cross(angular_rate, MOMENT_OF_INERTIA.dot(angular_rate))
        # sign is so that we take shortest path in S^3, probably only helps if our state is a quaternion :(
        quat_term = np.sign(attitude[0]) * self.quat_gain.dot(attitude_error[1:])
        rate_term = self.rate_gain.dot(angular_rate - rate_ref)
        reaction_wheels_command = - (-gyroscopic_term + quat_term + rate_term)
        #print(np.arccos(attitude_error[0])*2, attitude_error[1:], angular_rate - rate_ref)
        return reaction_wheels_command
