"""Test Quaternion Identities"""

import quaternion
import numpy as np
import timeit

import pytest

quat_zero = np.array([1, 0, 0, 0])

quat_p90x = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0, 0])
quat_p90y = np.array([np.sqrt(2)/2, 0, np.sqrt(2)/2, 0])
quat_p90z = np.array([np.sqrt(2)/2, 0, 0, np.sqrt(2)/2])

quat_n90x = np.array([np.sqrt(2)/2, -np.sqrt(2)/2, 0, 0])
quat_n90y = np.array([np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0])
quat_n90z = np.array([np.sqrt(2)/2, 0, 0, -np.sqrt(2)/2])

vect_px = np.array([1., 0, 0])
vect_py = np.array([0, 1., 0])
vect_pz = np.array([0, 0, 1])

vect_nx = np.array([-1., 0, 0])
vect_ny = np.array([0, -1., 0])
vect_nz = np.array([0, 0, -1.])


def make_parameters(steps, qfunc, pairs, checkfunc, invcheckfunc):
    """Make a list of parameters

    Parameters
    ----------
    steps
        individual quaternions to apply.
    qfunc
        quaternion function to combine two quaternions 'q_this = qfunc(q_step, q_prev)'
    pairs
        List of [original, solution] vector pairs for the quaternion at the end of the step
    checkfunc
        Function to transform original vector to solution 'solution == checkfunc(q_this, original)'
    invcheckfunc
        Function to transform solution vector back to original 'original = invcheckfunc(q_this, solution)'
    """
    # Build parameters
    parameters = []

    quat_i = quat_zero
    for i_idx, q_step in enumerate(steps):
        quat_i = qfunc(q_step, quat_i)
        for original, solution in pairs[i_idx]:
            parameters.append(
                [
                    quat_i, 
                    original, 
                    checkfunc, 
                    solution, 
                    invcheckfunc,
                ]
            )
    return parameters


# Create ids (explainations)
def make_ids(main_id, num_steps, num_cases):
    """
    Assumes that you have 3 cases per step
    """
    ids = []
    for step in range(num_steps):
        for case in range(num_cases):
            ids.append(main_id + "-step" + str(step+1) + "-case" + str(case+1))
    return ids



steps_1 = [quat_p90z, quat_p90x, quat_n90z, quat_p90y]
cases_1 = [
    [
        [vect_px, vect_py],
        [vect_py, -vect_px],
        [vect_pz, vect_pz],
    ],
    [
        [vect_px, vect_pz],
        [vect_py, -vect_px],
        [vect_pz, -vect_py],
    ],
    [
        [vect_px, vect_pz],
        [vect_py, vect_py],
        [vect_pz, -vect_px],
    ],
    [
        [vect_px, vect_px],
        [vect_py, vect_py],
        [vect_pz, vect_pz],
    ],
]

parameters_1 = make_parameters(
    steps_1, 
    quaternion.hamiltonian, 
    cases_1, 
    quaternion.h_sandwich, 
    quaternion.s_sandwich
)

ids_1 = make_ids("global-with-global-steps", len(steps_1), 3)

steps_2 = [quat_p90z, quat_n90y, quat_n90x, quat_p90y]
cases_2 = cases_1

parameters_2 = make_parameters(
    steps_2, 
    quaternion.shuster, 
    cases_2, 
    quaternion.h_sandwich, 
    quaternion.s_sandwich
)

ids_2 = make_ids("global-with-local-steps", len(steps_2), 3)


all_parameters = parameters_1 + parameters_2
all_ids = ids_1 + ids_2
@pytest.mark.parametrize("quaternion,vect_original,quat_func,vect_solution,inv_quat_func", all_parameters, ids=all_ids)
def test_operation(quaternion, vect_original, quat_func, vect_solution, inv_quat_func):
    """Check that that the quaternion functions and their inverses work as intended.

    Scalar first for testing please.
    quat_func(vect_orig) == vect_solution
    inv_quat_func(vect_solution) == vect_orig

    Parameters
    ----------
    quaternion
        Quaternion to use operations.
    vect_original
        Original vector to use.
    quat_function
        Quaternion sandwiching function to use (quat, vect)
    vect_solution
        The desired result of quat_func(vect_original)
    inv_quat_func
        The inverse quaternion sandwhiching function to use
    """
    def err(a, b):
        """Calculate a basic error."""
        return np.sum(np.abs(a-b))
    vect_transform = quat_func(quaternion, vect_original)
    try:
        assert err(vect_transform, vect_solution) < 1e-12
    except Exception as error:
        msg = str(vect_original) + " became " + str(vect_transform) + " instead of " + str(vect_solution)
        raise Exception(msg) from error
    inv_vect_transform = inv_quat_func(quaternion, vect_transform)
    try:
        assert err(inv_vect_transform, vect_original) < 1e-12
    except Exception as error:
        msg = str(vect_transform) + " inverted to " + str(inv_vect_transform) + " instead of " + str(vect_original)
        raise Exception(msg) from error


