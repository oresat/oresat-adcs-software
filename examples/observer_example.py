import numpy as np
import matplotlib.pyplot as plt

from oresat_adcs.classes import observers

from utility import helper

if __name__ == "__main__":

    # time between each measurement
    dt = 1.0
    # commands are first: magnetorquer (3) and reaction wheels (4)
    commands = [np.array([0, 0, 0]), np.array([0, 0, 0, 0])]

    # for true model 
    my_env = helper.environment.OrbitalEnvironment(hi_fi=True)
    my_sat = helper.make_satellite(my_env)
    my_dynamics = helper.dynamics.Dynamics(my_sat, my_env)
    my_true_state = helper.make_state()
    my_true_sim = helper.simulator.SimulatorDaemonInterface(my_dynamics, my_true_state, dt=0.5)


    # get initial measurements for kalman filter
    # the measurements have magnetorquer and sun vector things that we don't care about,
    # but its not hurting anyone (yet)
    init_state_list = my_true_sim.output(noisy=True)
    my_init_kalman_state = helper.dynamics.SatelliteState(np.array(init_state_list, dtype=object), my_true_state.clock)

    # check states
    #print(my_true_state, "\n\n", my_init_kalman_state)

    # pass the environement and initial state to kalman filter, it will use the state 
    # for the environment so it must have the first 5 numpy arrays in the state 
    my_position_filter = observers.DiscretePositionKalman(my_env, my_init_kalman_state)

    hey = []
    hey_m = []
    hey_f = []

    for iteration in range(1000):
        # MEASURE
        true_list = my_true_sim.output(noisy=False)
        measurement_list = my_true_sim.output(noisy=True)

        # UPDATE
        my_position_filter.update(measurement_list[0], measurement_list[1])

        # OUTPUT
        after_update = my_position_filter.output()

        # PROPOGATE
        # there may need to be multiple propogate steps
        my_position_filter.propagate(1.0)
        # update states locally
        my_position_filter.state.clock.tick(1.0)
        my_position_filter.state.update()

        # OUTPUT
        after_propagate = my_position_filter.output()

        # position
        #print(true_list[0], measurement_list[0], after_update[0], after_propogate[0]) 
        hey.append(true_list[0])
        hey_m.append(measurement_list[0])
        hey_f.append(after_update[0])
        # Propogate the simulation
        my_true_sim.propagate(1.0, commands)


    hey_array = np.array(hey).T
    hey_m_array = np.array(hey_m).T 
    hey_f_array = np.array(hey_f).T 

    #fig = plt.figure()
    #ax = plt.axes(projection='3d')
    #ax.plot3D(*hey_array, marker='.')
    #ax.plot3D(*hey_m_array, marker='.')
    #ax.plot3D(*hey_f_array, marker='.')

    #ax.set_title("Simulated GPS Position")
    #plt.show()

    # measurement error
    error_m = hey_m_array - hey_array
    # filter error
    error_f = hey_f_array - hey_array

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(*error_m, label='measurement error', marker=".")
    ax.plot3D(*error_f, label='filter error', marker=".")

    ax.legend()
    
    ax.set_title("Simulated GPS position error")
    plt.show()


    # do error magnitude over time
    error_m_mag = [np.linalg.norm(vect) for vect in error_m.T]
    error_f_mag = [np.linalg.norm(vect) for vect in error_f.T]

    fig = plt.figure()
    ax = plt.axes()
    ax.plot(range(len(error_m_mag)), error_m_mag, marker='.')
    ax.plot(range(len(error_f_mag)), error_f_mag, marker='.')
    plt.show()