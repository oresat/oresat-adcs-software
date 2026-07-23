import numpy as np
import matplotlib.pyplot as plt
import quaternion as quat
import guidance_utils as guid_utils


if __name__ == "__main__":
    sun_vector =  np.array([ 0.79026663, -0.56221362, -0.24370985])
    star_vector =  np.array([ 0.75628042,  0.63651392, -0.15129424])

    diag_solar_panels = np.array([1, 1, 0])
    star_tracker = np.array([-1, 0, 0])

    diag_solar_panels, star_tracker = guid_utils.orthonormalize_vectors(diag_solar_panels, star_tracker)
    sun_vector, star_vector = guid_utils.orthonormalize_vectors(sun_vector, star_vector)


    quat_1 = guid_utils.vector_transform_quat(
        a_vect = diag_solar_panels,
        b_vect = sun_vector,
    )

    quat_2 = guid_utils.two_boresights_quat(
        boresight_1 = diag_solar_panels,
        target_1 = sun_vector,
        boresight_2 = star_tracker,
        target_2 = star_vector
    )

    t_q1_v1 = quat.h_sandwich(quat_1, diag_solar_panels)
    t_q1_v2 = quat.h_sandwich(quat_1, star_tracker)
    t_q2_v1 = quat.h_sandwich(quat_2, diag_solar_panels)
    t_q2_v2 = quat.h_sandwich(quat_2, star_tracker)

    print(f"Quat 1: {quat_1}")
    print(f"Quat 2: {quat_2}")
    print(f"Transform q1 v1: {t_q1_v1}")
    print(f"Transform q1 v2: {t_q1_v2}")
    print(f"Transform q2 v1: {t_q2_v1}")
    print(f"Transform q2 v2: {t_q2_v2}")

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(projection="3d")

    ax1.plot(
        [0, diag_solar_panels[0]], 
        [0, diag_solar_panels[1]], 
        [0, diag_solar_panels[2]],
        label="Boresight 1"
    )
    ax1.plot(
        [0, star_tracker[0]], 
        [0, star_tracker[1]], 
        [0, star_tracker[2]],
        label="Boresight 2"
    )
    ax1.scatter(
        [0, sun_vector[0]], 
        [0, sun_vector[1]], 
        [0, sun_vector[2]],
        label="Target 1"
    )
    ax1.scatter(
        [0, star_vector[0]], 
        [0, star_vector[1]], 
        [0, star_vector[2]],
        label="Target 2"
    )
    ax1.plot(
        [0, t_q1_v1[0]], 
        [0, t_q1_v1[1]], 
        [0, t_q1_v1[2]],
        label="t q1 v1"
    )
    ax1.plot(
        [0, t_q1_v2[0]], 
        [0, t_q1_v2[1]], 
        [0, t_q1_v2[2]],
        label="t q1 v2"
    )
    ax1.plot(
        [0, t_q2_v1[0]], 
        [0, t_q2_v1[1]], 
        [0, t_q2_v1[2]],
        label="t q2 v1"
    )
    ax1.plot(
        [0, t_q2_v2[0]], 
        [0, t_q2_v2[1]], 
        [0, t_q2_v2[2]],
        label="t q2 v2"
    )
    plt.legend()
    plt.show()
