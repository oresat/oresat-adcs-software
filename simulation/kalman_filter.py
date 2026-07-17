from typing import Optional, Tuple

import numpy as np

from quaternion import axis_angle_to_quaternion, hemi, quat_conjugate, quat_mult


class MEKF:
    def __init__(
        self,
        p_star_tracker_0: float,
        sigma_star: float,
        p_b0: float,
        sigma_gyro: float,
        sigma_bias: float,
    ) -> None:
        # Computationally more efficient to define these once
        # rather than create them multiple times each iteration
        self.I3 = np.eye(3)
        self.Z3 = np.zeros((3, 3))
        self.I6 = np.eye(6)

        # save values for variable Q matrix definition
        self.sigma_star = sigma_star
        self.sigma_gyro = sigma_gyro
        self.sigma_bias = sigma_bias

        self.P_theta = p_star_tracker_0 * self.I3
        self.P_omega = p_b0 * self.I3
        self.P = None  # covariance matrix
        self.R = None  # measurement noise covariance matrix
        self.H = None  # Jacobian matrix of measurement model
        # estimated quaternion. Must be updated to initial measured quaternion in flight software
        self.q = None
        # last gyro measurement to perform prediction
        # using body rates of last update step omega_{k-1}
        # Must be updated to initial measured rate in flight software
        self.last_omega = None
        self.b = np.zeros(3)  # estimated gyro bias (3x1)

        # time of last prediction step execution
        # needs to be reinitialized to current time
        # whenever state estimation is switched from inactive to active
        self.last_time = 0
        # track whether IMU data was part of last step
        # for decision on Zero-Order_Hold (ZOH) or Midpoint Rule usage
        self.prev_event_used_IMU = True

    def reset(self, q_init, omega_init, time_init) -> None:
        self.q = q_init
        self.last_omega = omega_init
        self.last_time = time_init
        self.prev_event_used_IMU = True

        self.b = np.zeros(3)
        self.R = self.sigma_star**2 * self.I3
        self.H = 0.3 * np.eye(3, 6)
        self.P = np.block([[self.P_theta, self.Z3], [self.Z3, self.P_omega]])

    def update(
        self,
        current_time: float,
        omega: Optional[np.ndarray] = None,
        q_measured: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Update Kalman filter and return output (event based function)

        Parameters
        ----------
        current_time : float
            Current time, in seconds
        omega : Optional[np.ndarray]
            Gyroscope rate measurement
        q_measured : Optional[np.ndarray]
            Star tracker orientation (quaternion)

        Returns
        -------
        Tuple[np.ndarray, np.ndarray]
            A tuple containing the estimated quaternion and bias-corrected rate measurement
        """
        dt = current_time - self.last_time

        # improve accuracy of prediction if new omega is available in order to form midpoint rate
        if omega is not None:
            if self.prev_event_used_IMU:
                # predict state using midpoint rule if last prediction didn't use ZOH
                self.prediction(dt, 0.5 * (self.last_omega + omega))
            else:
                self.prediction(dt, self.last_omega)  # predict state using ZOH
            # if gyro sensor measurement exists, we update the saved gyro values for the next step
            self.last_omega = omega
            self.prev_event_used_IMU = True
        else:
            self.prediction(dt, self.last_omega)  # predict state using ZOH
            self.prev_event_used_IMU = False

        # if star tracker sensor measurement exists, perform correction step as well
        if q_measured is not None:
            self.correction(q_measured)

        # protect against calls without new information
        if omega is not None or q_measured is not None:
            self.last_time = current_time  # update last time a prediction was performed

        # return filtered attitude estimate and bias-correct rate measurement
        return self.q, (self.last_omega - self.b)

    def prediction(self, dt: float, omega: np.ndarray) -> None:
        """Predict next state based on IMU input. Also known as the propagation or estimation step.

        Parameters
        ----------
        dt : float
            Time delta
        omega : np.ndarray
            Gyroscope rate measurement
        """
        omega = omega - self.b  # correct omega with estimated gyro bias
        phi = self.phi_matrix(dt, omega)

        # propagate estimated quaternion state based on body rates
        # using the exponential map integration of the quaternion kinematic equation
        theta = np.linalg.norm(omega) * dt
        half = 0.5 * theta
        if theta > 1e-8:
            s = np.sin(half) / theta
        else:
            # avoid precision loss using series for small angles
            s = 0.5 - theta**2 / 48.0
        c = np.cos(half)
        delta_q = np.r_[s * omega * dt, c]
        delta_q /= np.linalg.norm(delta_q)
        self.q = quat_mult(delta_q, self.q)

        q = self.q_matrix(dt)
        self.P = phi @ self.P @ phi.T + q  # update covariance matrix

    def correction(self, q_measured: np.ndarray) -> None:
        """Correct/update filter based on measurement input from star tracker.
        Also known as the innovation or update step.

        Parameters
        ----------
        q_measured : np.ndarray
            Star tracker orientation measurement (quaternion)
        """
        # calculate the innovation quaternion (measurement residual)
        # REQUIRES HEMI FUNCTION. Without hemi, massive errors arise
        q = hemi(quat_mult(q_measured, quat_conjugate(self.q)))
        y = 2 * q[:3]  # small-angle innovation vector

        # define Kalman gain
        k = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        dx = k @ y
        d_theta = dx[:3]
        db = dx[3:]

        # update quaternion
        theta = np.linalg.norm(d_theta)
        if theta < 1e-12:
            delta_q = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            axis = d_theta / theta
            delta_q = axis_angle_to_quaternion(axis, theta)  # scalar-last
        self.q = hemi(quat_mult(delta_q, self.q))

        self.b += db  # update gyro bias (bias correction)

        # covariance Joseph update
        i6_h = self.I6 - k @ self.H
        self.P = i6_h @ self.P @ i6_h.T + k @ self.R @ k.T  # update covariance matrix
        # enforce symmetry after the Joseph update to kill numerical skew
        self.P = 0.5 * (self.P + self.P.T)

        # MEKF reset mapping (new error coordinates)
        # Gamma ~ I - 0.5*[d_theta]X <- skew matrix built from d_theta
        theta_skew = self.skew(d_theta)
        gamma = self.I3 - 0.5 * theta_skew
        g = np.block([[gamma, self.Z3], [self.Z3, self.I3]])
        self.P = g @ self.P @ g.T

    def phi_matrix(self, dt: float, omega: np.ndarray) -> np.ndarray:
        skew_matrix = self.skew(omega)
        s2 = skew_matrix @ skew_matrix
        norm = np.linalg.norm(omega)

        if norm < 1e-6:
            phi11 = self.I3 - skew_matrix * dt + 0.5 * s2 * dt**2
            phi12 = -self.I3 * dt + 0.5 * skew_matrix * dt**2 - (1.0 / 6.0) * s2 * dt**3
        else:
            w1, w2, w3 = norm, norm**2, norm**3
            phi11 = self.I3 - skew_matrix * np.sin(w1 * dt) / w1 + s2 * (1 - np.cos(w1 * dt)) / w2
            phi12 = (
                skew_matrix * (1 - np.cos(w1 * dt)) / w2
                - self.I3 * dt
                - s2 * (w1 * dt - np.sin(w1 * dt)) / w3
            )

        phi = np.block([[phi11, phi12], [self.Z3, self.I3]])
        return phi

    def q_matrix(self, dt: float) -> np.ndarray:
        q11 = (self.sigma_gyro**2 * dt + (self.sigma_bias**2) * dt**3 / 3.0) * self.I3
        q12 = (-(self.sigma_bias**2 * dt**2 / 2.0)) * self.I3
        q22 = (self.sigma_bias**2 * dt) * self.I3
        return np.block([[q11, q12], [q12, q22]])

    @staticmethod
    def skew(omega) -> np.ndarray:
        """Return the 3x3 skew-symmetric matrix (cross-product matrix)
        of a 3-element angular velocity vector omega.
        """
        wx, wy, wz = omega
        return np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
