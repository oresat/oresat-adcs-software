import numpy as np
# various physical constants and parameters

earth_rotation = 7.2921158553e-5 # rad/s, rotation of Earth around axis
G_newton = 6.6743e-11 #m^3 kg^-1 s^-2
oresat_mass = (21.66*4 + 2272.07) / 1000 # kg, mass of oresat
mu = G_newton * oresat_mass # gravitational parameter of body

# for geocentric magnetic field model
# see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
mag_ref_radius = 6.3712e6 # m, magnetic spherical ref. radius
# these coefficients are in nT. from IGRF-13, 2020.0
g_1_1 = -1450.9 # secular variation 7.4 / year
h_1_1 = 4652.5 # secular variation -25.9 / year
g_1_0 = -29404.8 # secular variation 5.7 / year

# reaction wheel subsystem
# spin axes of wheels, starts at 5 because facing inward
S_rw = np.sin(np.pi / 3)
C_rw = np.cos(np.pi / 3)
wheel_axes = np.array([[S_rw * np.cos(np.pi * (2 * n + 5) / 4),
                        S_rw * np.sin(np.pi * (2 * n + 5) / 4),
                        C_rw] for n in range(4)])

# this matrix distributes angular velocity of wheel system into angular velocity of wheel components
# the pseudoinverse method minimizes norm of torque/momentum vector which is sum of individual wheels
# there is another method for minimizing max effort of any wheel (minimax)
# right now we don't care, but we may later
rwsys_to_wheels = np.linalg.pinv(wheel_axes.T)

# moments of inertia, all in kg * m^2 and body coordinates
# body moment does NOT include reaction wheels
body_moment = np.array([[1.109521565e-2, 0, 0],
                    [0, 1.109521565e-2, 0],
                    [0, 0, 5.09404743e-3]])
rw_moment_perpend = np.ones(4) * 1.02562e-6
rw_moment_parallel = np.ones(4) * 1.64023e-6 # only this is relevant to wheel control

# moment of inertia for satellite and parts of wheels orthogonal to spin axes
def find_J_B(J_B_hat, axes, J_perps):
    id3 = np.identity(3)
    wheels = sum([J_perps[i] * (id3 - np.outer(e, e)) for i, e in enumerate(axes)])
    return J_B_hat + wheels

moment_of_inertia = find_J_B(body_moment, wheel_axes, rw_moment_perpend)
Cd = 2 # guessed drag coefficient, could be anywhere from 1 - 2.5


# dynamic model initial conditions
x_0 = np.array([1.91831688780483e6, 6.52089247589002e6, 1.94903609076208e3]) # m. this and v are from freeflyer
v_0 = np.array([-4.56009030296681e3, 1.33278201869975e3, 6.0067784327447e3]) # m/s
q_0 = np.array([1, 0, 0, 0])
w_0 = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
whl_0 = np.array([0, 0, 0, 0])
cur_0 = np.array([0, 0, 0])
t_0 = (2020, 9, 1, 0, 0, 0)
