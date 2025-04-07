
import numpy as np

from oresat_adcs.classes import jday, dynamics, sensor
from oresat_adcs.configuration import environment, structure
from oresat_adcs.system import simulator

def make_satellite(my_env):

    my_instruments= [structure.SensitiveInstrument(np.array([0, 0, -1]), bounds=[15, 100], forbidden=[True, False], obj_ids=[0]),
                     structure.SensitiveInstrument(np.array([0, -1, 0]), bounds=[180, 180], forbidden=[False, False], obj_ids=[])]


    # make torqer system
    linearized = True
    mt_type = "LinearRod" if linearized else "Rod"
    torquers = [structure.Magnetorquer(mt_type, np.array([1, 0, 0]), 0.0625),
                structure.Magnetorquer(mt_type, np.array([0, 1, 0]), 0.0625),
                structure.Magnetorquer("Square", np.array([0, 0, 1]), 0.2887)]
    my_mt_system = structure.MagnetorquerSystem(torquers)

    # make reaction wheel system
    inclination = np.pi / 3
    azimuth = np.pi / 4
    parallel_moment = 1.64023e-6
    orthogonal_moment = 1.02562e-6
    max_T = 0.0005
    torque_limited = True
    given_axes = [np.array([np.sin(inclination)*np.cos(azimuth+rw_index*np.pi/2),
                            np.sin(inclination)*np.sin(azimuth+rw_index*np.pi/2),
                            np.cos(inclination)]) for rw_index in range(4)]
    wheels = [structure.Wheel(given_axes[0], parallel_moment, orthogonal_moment),
              structure.Wheel(given_axes[1], parallel_moment, orthogonal_moment),
              structure.Wheel(given_axes[2], parallel_moment, orthogonal_moment),
              structure.Wheel(given_axes[3], parallel_moment, orthogonal_moment)]
    my_rw_system = structure.ReactionWheelSystem(wheels, max_T, torque_limited)


    # MKS units
    mass = (21.66*4 + 2419.56) / 1000
    absorption = 0.84
    drag = 2
    principal_moments = np.array([1.378574142e-2,
                                  1.378854578e-2,
                                  5.49370596e-3])
    # products of inertia: xy, xz, yz
    products_of_inertia = principal_moments*0.1
    dimensions = np.array([0.1, 0.1, 0.2])


    my_sensors = [sensor.GPS_pos(mean=0, std_dev=30, env=my_env),
               sensor.GPS_vel(mean=0, std_dev=2, env=my_env),
               sensor.StarTracker(mean=0, std_dev=0.75e-7, env=my_env, size=4),
               sensor.Gyro(arw_mean=0, arw_std_dev=2.79e-4, 
                                rrw_mean=0, rrw_std_dev=8.73e-7, 
                                init_bias=3.15e-5, env=my_env),
               sensor.Wheel_vel(mean=0, std_dev=0.0001, env=my_env, size=4),
               sensor.Magnetometer(mean=0, std_dev=4e-8, env=my_env), # from datasheet
               sensor.SunSensor(mean=0, std_dev=1e-6, env=my_env)
                            ]
    my_satellite = structure.Satellite(mass=3.0,
                                        dimensions=np.array([0.1, 0.1, 0.2]),
                                        absorption=0.84,
                                        drag_coeff=1.0,
                                        principal_moments=principal_moments,
                                        product_moments=products_of_inertia,
                                        reduced=False,
                                        sensors=my_sensors,
                                        rw_sys=my_rw_system,
                                        mt_sys=my_mt_system,
                                        sensitive_instruments=my_instruments)
    

    return my_satellite


def run_simulations(simulator, commands, total_iterations):
    output = ""
    for iteration in range(total_iterations):
        out = simulator.propagate(2, commands)

        output += (",".join([str(component) for component in out[0]])) + "\n"

        if iteration % 10 == 0:
            print("Iteration ", iteration, " of ", total_iterations," complete")

    print(output)



if __name__ == "__main__":
    # Init a dynamical model without a satellite (uses default satellite)
    x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
    v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
    q_0   = np.array([1, 0, 0 ,0])
    w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
    whl_0 = np.array([300, -300, -300, 300]) * 0
    t_0 = (2024 ,7, 7, 14, 0, 0)
    dt    = 0.05 # perhaps we want to choose this upstream?

    my_jclock = jday.JClock(*t_0)
    my_state = dynamics.SatelliteState(np.array([x_0, v_0, q_0, w_0, whl_0], dtype=object), my_jclock)

    my_env = environment.OrbitalEnvironment(hi_fi=True)
    my_satellite = make_satellite(my_env)

    # Create the dynamics
    my_dynamics = dynamics.Dynamics(my_satellite, my_env)

    # Create the simulation
    my_simulator = simulator.SimulatorDaemonInterface(my_dynamics, my_state, dt)

    # try some simulation
    #the maximum values for MTs are 60,000 for x and y and 300,000 for z
    cmds = [np.array([0, 0, 300000]), np.array([0,0,0,0])]

    # format simulation output quaternion to be like a csv
    run_simulations(my_simulator, cmds, 100)

    print("\nDone")




