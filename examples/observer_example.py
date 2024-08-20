import numpy as np

from oresat_adcs.classes import new_observer

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
    my_init_kalman_state = helper.dynamics.SatelliteState(np.array(init_state_list, dtype=object))
    my_init_kalman_state.attach_clock(my_true_state.clock)
    my_init_kalman_state.update()

    # check states
    print(my_true_state, "\n\n", my_init_kalman_state)
    # pass the environement and initial state to kalman filter, it will use the state 
    # for the environment so it must have the first 5 numpy arrays in the state 
    my_position_filter = new_observer.DiscretePositionKalman(my_env, my_init_kalman_state)


    for _ in range(5):
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
        after_propogate = my_position_filter.output()

        # position
        print(true_list[0], measurement_list[0], after_update[0], after_propogate[0]) 
        # Propogate the simulation
        my_true_sim.propagate(1.0, commands)