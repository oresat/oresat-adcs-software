import numpy as np
import geometry # can't remember whether these imports make namespace "flat"
import system_definition
import dynamic_model

# Point and Stare Maneuver performed in inertial frame (because attitude is inertial)
# convert coordinates from ECEF to ECI before calling
# If we want to do a fancier tracking move, it will require some thought
# returns the rotation quaternion for pointing a camera at target
def acquire_target(target_I, position, q_BI, camera_vec):
    # this unit vector points from the sat to an arbitrary target
    def pointing_vector(target_I, r_I):
        P_I = target_I - r_I
        return P_I / np.linalg.norm(P_I)

    # returns parameters for an active rotation of camera vector
    # towards pointing vector with fixed inertial frame
    def R_axis_angle(camera, direction):
        r = np.cross(camera, direction)
        # be aware that this might crash if float error gives a dot product > 1 or < -1
        theta = np.arccos(np.dot(camera, direction))
        return r, theta # use + for active (- for passive)

    P_I = pointing_vector(target_I, position) # pointing vector in inertial frame
    camera_I = sandwich(conjugate(q_BI), camera_vec) # expression of camera direction in inertial frame
    r, theta = R_axis_angle(camera_I, P_I) # transform from inertial to desired
    return axisangle_to_quat(r, theta) # active rotation that takes cam to P in fixed inertial frame


# see Attitude Control Simulator for the Small Satellite and Its Validation by On-orbit Data of QSAT-EOS
# for descriptions of next couple functions
# take shortest path to correct errors. keep in mind that S^3 double-covers SO(3)
def error_quat(q_true, q_ref):
    q = quat_product(conjugate(q_ref), q_true)
    if q[0] < 0:
        return -q
    else:
        return q

# given instanteous target, give ideal wheel accelerations to be pointed at it
def point_and_stare(body_ang_vel, target_I, position, q_BI, camera_vec, steady_state_error):
    attitude_error = error_quat(q_BI,
                                acquire_target(target_I, position, q_BI, camera_vec))
    vel_error = attitude_derivative(attitude_error, body_ang_vel)[1:] - body_ang_vel
    steady_state_error += attitude_error[1:] * 0.001 # this is an arbitrary multiplier for now
    return RWSYS_TO_WHEELS.dot(-P_COEFF * attitude_error -
                                I_COEFF * steady_state_error -
                                D_COEFF * vel_error), steady_state_error
        # I don't know whether they should all be negative, needs thought
