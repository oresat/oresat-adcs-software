import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from adcs_lib import quaternion
from mpl_toolkits.mplot3d import Axes3D

# this class is just for my benefit of graphing data after simulations.

true   = pd.read_csv('./true_states.csv')
kalman = pd.read_csv('./kalman_states.csv')
start = None
end = None
endt = None if end is None else end + 1
absolute = True
#fig, (ax1, ax2, ax3, ax4) = plt.subplots(4)

#ax1.plot(true.loc[:100, ['x_x', 'x_y', 'x_z']])
#ax1.plot(kalman.loc[:100, ['x_x', 'x_y', 'x_z']])
t = [0.05 * i for i in range(len(kalman.loc[:, ['x_x', 'x_y', 'x_z']]))]
J_list = [P*0.05 for P in true.loc[start:end, 'P']]
J = sum(J_list)
print(J,"J consumed total")
print(np.average(true.loc[start:end, 'P']), "W spent on average")
print(np.average((true.loc[start:end, ['w_x', 'w_y']])), "rad/s average x and y rate")
print(np.average((true.loc[start:end, ['w_z']])), "rad/s average z rate")
#print(np.min(true.loc[start:end, ['I']]))

plt.figure()
#print(true.loc[-1:None, ['x_x', 'x_y', 'x_z', 'v_x', 'v_y', 'v_z']])
plt.subplot(211)
if absolute:
    plt.plot(t[start:endt], true.loc[start:end, ['x_x', 'x_y', 'x_z']], linestyle='dashed')
    plt.plot(t[start:endt], kalman.loc[start:end, ['x_x', 'x_y', 'x_z']], linestyle='dotted')
else:
    plt.plot(t[start:endt], true.loc[start:end, ['x_x', 'x_y', 'x_z']] - kalman.loc[start:end, ['x_x', 'x_y', 'x_z']], linestyle='solid')

plt.subplot(212)
if absolute:
    plt.plot(t[start:endt], true.loc[start:end, ['v_x', 'v_y', 'v_z']], linestyle='dashed')
    plt.plot(t[start:endt], kalman.loc[start:end, ['v_x', 'v_y', 'v_z']], linestyle='dotted')
else:
    plt.plot(t[start:endt], true.loc[start:end, ['v_x', 'v_y', 'v_z']] - kalman.loc[start:end, ['v_x', 'v_y', 'v_z']], linestyle='solid')


plt.figure()

plt.subplot(211)
if absolute:
    plt.plot(t[start:endt], true.loc[start:end, ['q0', 'q1', 'q2', 'q3']], linestyle='dashed')
    #plt.plot(t[start:endt], true.loc[start:end, ['qd0', 'qd1', 'qd2', 'qd3']], linestyle='dotted')
    #plt.plot(t[start:endt], kalman.loc[start:end, ['roll', 'pitch', 'yaw']], linestyle='solid')
    #plt.plot(t[start:endt], kalman.loc[start:end, ['q0', 'q1', 'q2', 'q3']], linestyle='dashed')
    plt.ylabel('Quaternions')
else:
    #plt.plot(t[start:endt], true.loc[start:end, ['q0', 'q1', 'q2', 'q3']] - kalman.loc[start:end, ['q0', 'q1', 'q2', 'q3']], linestyle='solid')
    #plt.ylabel('Quaternion Error')
    plt.plot(t[start:endt], kalman.loc[start:end, ['roll', 'pitch', 'yaw']], linestyle='solid')
    plt.ylabel('Euler angle error')
plt.xlabel('Time (s)')

plt.subplot(212)
if absolute:
    plt.plot(t[start:endt], true.loc[start:end, ['w_x', 'w_y', 'w_z']], linestyle='dashed')
    #plt.plot(t[start:endt], kalman.loc[start:end, ['w_x', 'w_y', 'w_z']], linestyle='dotted')
    plt.ylabel('Angular velocity (rad/s)')
else:
    plt.plot(t[start:endt], true.loc[start:end, ['w_x', 'w_y', 'w_z']] - kalman.loc[start:end, ['w_x', 'w_y', 'w_z']], linestyle='solid')
    plt.ylabel('Angular velocity Error (rad/s)')
plt.xlabel('Time (s)')
plt.show()

plt.subplot(111)
if absolute:
    plt.plot(t[start:endt], true.loc[start:end, ['W1', 'W2', 'W3', 'W4']], linestyle='dashed')
    #plt.plot(t[start:endt], kalman.loc[start:end, ['W1', 'W2', 'W3', 'W4']], linestyle='dotted')
    plt.ylabel('Wheel Angular velocity (rad/s)')
else:
    plt.plot(t[start:endt], true.loc[start:end, ['W1', 'W2', 'W3', 'W4']] - kalman.loc[start:end, ['W1', 'W2', 'W3', 'W4']], linestyle='solid')
    plt.ylabel('Wheel Angular velocity Error (rad/s)')
plt.xlabel('Time (s)')

plt.show()

plt.figure()
plt.subplot(211)
plt.plot(t[start:endt], true.loc[start:end, 'P'])
plt.ylabel('Power (W)')
plt.xlabel('Time (s)')

plt.subplot(212)
plt.plot(t[start:endt], true.loc[start:end, ['I']], linestyle='dashed')
plt.ylabel('Magnetorquer Current  (A)')
plt.xlabel('Time (s)')

plt.show()

s = np.array(true.loc[start:end, ['S_x', 'S_y', 'S_z']])[0]
s = quaternion.sandwich_opp(np.array(true.loc[start:end, ['q0', 'q1', 'q2', 'q3']])[0], s)
sun = [np.mod(np.degrees(np.arctan2(s[1], s[0])), 360), np.degrees(np.arcsin(s[2]/np.linalg.norm(s)))]
cams = [quaternion.sandwich_opp(q, [0, 0, -1]) for q in np.array(true.loc[start:end, ['q0', 'q1', 'q2', 'q3']])]
cam = [[np.mod(np.degrees(np.arctan2(c[1], c[0])), 360), np.degrees(np.arcsin(c[2]/np.linalg.norm(c)))] for c in cams]
#cam = [[np.mod(np.degrees(np.arctan2(c[1], c[0])), 360), np.mod(np.degrees(np.arcsin(c[2]/np.linalg.norm(c))), 360)] for c in cams]
#radec = [quaternion._quat2equatorial(q)[:2] for q in np.array(true.loc[start:end, ['q0', 'q1', 'q2', 'q3']])]
fig, ax = plt.subplots()
ax.scatter([x[0] for x in cam], [x[1] for x in cam], s=0.1)
circle = plt.Circle(sun, 15, color='r',alpha=.5)
ax.add_artist(circle)
plt.xlabel('Right Ascension (deg)')
plt.ylabel('Declination (deg)')
plt.show()
