
import numpy as np
import quaternion as quat
from discrete_state_space_controller import get_gain_matrix

# Gain mode is for controller states
import numpy as np
from config import GainModeRW
from typing import TypedDict


class MTController():
    """Magnetorquer controller, assumes 3-axis orthogonal control"""
    def __init__(self, detumble_gain):
        self._detumble_gain = detumble_gain

    def get_detumble_torque(self, omega):
        """Calculates the detumble torque.
        Based on a controller by Markley and Crassidis"""
        return self._detumble_gain * omega

    def get_desaturation_torque(self, h_wheels):
        """Calculates the torque to do momentum dumping of wheels"""
        # TODO: temporarily use detumble gain, but see if another type of gain works.
        return self._detumble_gain * h_wheels
    def get_dipoles(self, torque, mag_field):

        # version 1

        # detumble controller as defined by Markley & Crassidis
        # tau_des = detumble_gain * omega
        desired_dipole = np.cross(torque, mag_field) / (mag_field @ mag_field)
        


        # version 3

        #tau_des = self.mag_LQR_controller(q_error, omega) # desired 3-axis torque in body frame
        #bm = self.b_mat(B)
        #k = 1e-8

        #m_cmd_lqr = np.linalg.inv(bm.T @ bm + k*np.eye(3))@bm.T@tau_des



    def b_mat(self, mag_vector: np.ndarray) -> np.ndarray:
        """Turn the magnetic field vector into a torque matrix."""
        bx, by, bz = b
        return np.array([[0, bz, -by], [-bz, 0, bx], [by, -bx, 0]])


    def dipole_saturation(self, desired_dipoles):
        """Scale dipole vector to meet current requirements."""
        pass

