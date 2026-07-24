
import numpy as np
import quaternion as quat
from discrete_state_space_controller import get_gain_matrix

# Gain mode is for controller states
import numpy as np
from config import GainModeRW
from typing import TypedDict


class RWController():
    def __init__(
        rw_inertia: np.ndarray,
        use_variable_gain: bool,
        sat_inertia: np.ndarray,
        update_time: float,
        lqr_max_input: float = 0.001,
        lqr_max_error: float = 1,
        lqr_max_rate: float = 0.09,
        lqr_fine_max_input: float = 0.01,
        lqr_fine_max_error: float = 0.05,
        lqr_fine_max_rate: float = 0.2,
    ):
        """
        Parameters
        ----------
        rw_inertia
            Inertia matrix
        use_variable_gain
            Enable to switch between controllers
        sat_inertia
            Satellite inertia matrix
        update_time
            Iteration update interval
        """
        self.rw_inertia = rw_inertia
        self.use_variable_gain = use_variable_gain
        self.sat_inertia = sat_inertia
        self.update_time = update_time

        # Create gain matrixes
        self._update_lqr_standard(
            sat_inertia = sat_inertia,
            update_time = update_time,
            max_error = lqr_max_error,
            max_rate = lqr_max_rate,
            max_input = lqr_max_input,
        )

        # Create gain matrixes
        self._update_lqr_fine(
            sat_inertia = self.sat_inertia,
            update_time = self.update_time,
            max_error = lqr_fine_max_error,
            max_rate = lqr_fine_max_rate,
            max_input = lqr_fine_max_input,
        )

        # Internal gain mode states and parameters
        self._gain_mode = GainModeRW.STANDARD
        self.transient_time_s = 30
        self.transient_start = 0
 


    def _update_lqr_standard(
        self, sat_inertia, update_time, max_error, max_rate, max_input
    ):
        """Calculate a new gain matrix for the standard controller."""
        self.K_RW = get_gain_matrix(
            j=sat_inertia
            timestep=update time
            max_error=max_error
            max_input=max_rate
            max_input=max_input
        )

    def _update_lqr_fine(
        self, sat_inertia, update_time, max_error, max_rate, max_input
    ):
        """Calculate a new gain matrix for the fine controller."""
        self.K_RW_fine = get_gain_matrix(
            j=sat_inertia
            timestep=update time
            max_error=max_error
            max_input=max_rate
            max_input=max_input
        )



    def control(
        self, q_error: np.ndarray, omega: np.ndarray, curren_time_secs: float
    ):
        # assemble state vector
        x = np.concatenate((q_error[:3], omega))

        if self.use_variable_gain and (quat.error_angle(q_error) < 1): 
            # LQR controller with integral term
            transient_time = 30 # seconds
            if self._gain_mode == GainModeRW.STANDARD:
                self.transient_start = currentTimeSecs
                # switch to transient mode
                self._gain_mode = GainModeRW.TRANSIENT
                # first step of transient mode returns same value as
                # standard controller
                return - self.K_RW @ x

            elif self._gain_mode == GainModeRW.TRANSIENT:
                # Transient gain mode
                if (self.transient_start >= self.transient_start+transient_time):
                    self._gain_mode = GainModeRW.FINE_POINTING # switch to full fine-pointing mode
                gain_switch_time = currentTimeSecs - self.transient_start
                return (-self.K_RW_fine @ x)*gain_switch_time/transient_time - (self.K_RW @ x)*(1-gain_switch_time/transient_time) # transient mode

            else:
                # Fine pointing gain mode
                # - self.K_integrator @ self.state_integral
                return -self.K_RW_fine @ x 
        else:
            # use standard gain method
            self._gain_mode = GainModeRW.STANDARD
            # invert sign for control
            return -self.K_RW @ x
        


