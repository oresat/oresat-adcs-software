import numpy as np
from geometry import *
# various physical constants and parameters

EARTH_ROTATION = 7.2921158553e-5 # rad/s, rotation of Earth around axis
G_NEWTON = 6.6743e-11 #m^3 kg^-1 s^-2
EARTH_MASS = 5.972e24 # kg
ORESAT_MASS = (21.66*4 + 2272.07) / 1000 # kg, mass of oresat
#EARTH_MU = 3.986004418e14 # m^3 s^-2, standard gravitational parameter of earth
MU = G_NEWTON * (ORESAT_MASS + EARTH_MASS) # gravitational parameter of body wrt earth

# for geocentric magnetic field model
# see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
MAG_REF_RADIUS = 6.3712e6 # m, magnetic spherical ref. radius
# these coefficients are in nT. from IGRF-13, 2020.0
G_1_1 = -1450.9 # secular variation 7.4 / year
H_1_1 = 4652.5 # secular variation -25.9 / year
G_1_0 = -29404.8 # secular variation 5.7 / year

# reaction wheel subsystem
# spin axes of wheels, starts at 5 because facing inward
S_rw = np.sin(np.pi / 3)
C_rw = np.cos(np.pi / 3)
WHEEL_AXES = np.array([[S_rw * np.cos(np.pi * (2 * n + 5) / 4),
                        S_rw * np.sin(np.pi * (2 * n + 5) / 4),
                        C_rw] for n in range(4)])

# this matrix distributes angular velocity of wheel system into angular velocity of wheel components
# the pseudoinverse method minimizes norm of torque/momentum vector which is sum of individual wheels
# there is another method for minimizing max effort of any wheel (minimax)
# right now we don't care, but we may later
RWSYS_TO_WHEELS = np.linalg.pinv(WHEEL_AXES.T)
# I emphasise these coefficients are entirely arbitrary
P_COEFF = 0.5
I_COEFF = 0.01
D_COEFF = 10

# moments of inertia, all in kg * m^2 and body coordinates
# body moment does NOT include reaction wheels
body_moment = np.array([[1.109521565e-2, 0, 0],
                    [0, 1.109521565e-2, 0],
                    [0, 0, 5.09404743e-3]])
rw_moment_perpend = np.ones(4) * 1.02562e-6
RW_MOMENT_PARALLEL = np.ones(4) * 1.64023e-6 # only this is relevant to wheel control

# moment of inertia for satellite and parts of wheels orthogonal to their spin axes
def find_J_B():
    wheel_moments = sum([rw_moment_perpend[i] * (np.identity(3) - np.outer(axis, axis))
                 for i, axis in enumerate(WHEEL_AXES)])
    return body_moment + wheel_moments

# note that the total moment of inertia is MOMENT_OF_INERTIA + WHEEL_MOMENT
# refer to section 3.3.5 when confused
MOMENT_OF_INERTIA = find_J_B()
INV_MOMENT_OF_INERTIA = np.linalg.inv(MOMENT_OF_INERTIA)
WHEEL_MOMENT = sum([RW_MOMENT_PARALLEL[i] * (np.outer(axis, axis))
                    for i, axis in enumerate(WHEEL_AXES)])
DRAG_COEFF = 2 # guestimated drag coefficient, could be anywhere from 1 - 2.5

# kalman filter constants, for a rainy day
BIAS_SIGMA = 0.05236 # rad/s initial bias std dev, guestimate
GYRO_SIGMA = 5.818 * 10**(-4) # angular random walk, rad/sqrt(s), from datasheet
DRIFT_SIGMA = 1.713 * 10**(-8) # rate random walk, rad / s^(3/2), guestimate
SENSOR_SIGMA = 1.5 * 10**(-5) # rad, star tracker measurement noise, guestimate
g = GYRO_SIGMA**2
d = DRIFT_SIGMA**2
# untuned process noise model, assuming variance is independent
Q_MATRIX = np.array([
        [g, 0, 0, 0, 0, 0],
        [0, g, 0, 0, 0, 0],
        [0, 0, g, 0, 0, 0],
        [0, 0, 0, d, 0, 0],
        [0, 0, 0, 0, d, 0],
        [0, 0, 0, 0, 0, d]
        ])
# change in linear EKF with respect to noise
G_MATRIX = np.array([
        [-1, 0, 0, 0, 0, 0],
        [0, -1, 0, 0, 0, 0],
        [0, 0, -1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1.]
        ])
# measurement covariance
R_MATRIX = np.diag([SENSOR_SIGMA**2 for i in range(3)])

# dynamic model initial conditions
x_0 = np.array([1.91831688780483e6, 6.52089247589002e6, 1.94903609076208e3]) # m. this and v are from freeflyer
v_0 = np.array([-4.56009030296681e3, 1.33278201869975e3, 6.0067784327447e3]) # m/s
q_0 = np.array([1, 0, 0, 0])
w_0 = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
whl_0 = np.array([0, 0, 0, 0])
cur_0 = np.array([0, 0, 0])
t_0 = (2020, 9, 1, 0, 0, 0)
