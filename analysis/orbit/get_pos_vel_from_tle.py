import requests
import json
import math
import numpy

def get_tle(satnogs_id="DKCD-1609-0567-7056-3922"):
    tle_url = f"https://db.satnogs.org/api/tle/?format=json&sat_id={satnogs_id}"
    result = requests.get(tle_url)
    result = json.loads(result.text)[0]
    return result["tle1"] , result["tle2"]


if __name__ == "__main__":
    #tle1, tle2 = get_tle()

    #tle1_elements = tle1.split()
    #tle2_elements = tle2.split()

    #print(tle1_elements)

    # nu is the true anomaly
    # M is the mean anomaly
    # M_init is a reference anomaly
    # n is the mean motion
    n_mean_motion = 15.188 * 2 * math.pi / 86400 #rad/s

    # gravitational constant times mass of earth
    mu = 3.9860 * 10**14

    # in degrees, convert to radians for sin/cos
    M_mean_anomaly = 221.6340 * math.pi / 180


    # if time has passed since epoch, update the mean anomaly
    #M_mean_anomaly = M_init + n(delta_t)

    # Use newtons method
    # recursively find E

    e_ecc = 0.0009895
    
    E_hat = M_mean_anomaly

    for i in range(5):
        delta_E = (M_mean_anomaly - (E_hat - e_ecc*math.sin(E_hat)))/(1-e_ecc*math.cos(E_hat))
        E_hat = E_hat + delta_E
        #print(delta_E, E_hat)

    a = (mu**(1/3)) / (n_mean_motion**(2/3))
    #print(a)
    r_radius = a*(1 - math.e*math.cos(E_hat))

    #print(r_radius)

    x_position = a*(math.cos(E_hat) - e_ecc)
    y_position = a*math.sqrt(1-(e_ecc**2))*math.sin(E_hat)

    x_dot_position = -(n_mean_motion * a**2 / r_radius) * math.sin(E_hat)
    y_dot_position = (n_mean_motion * a**2 / r_radius) * math.sqrt(1-(e_ecc**2))*math.sin(E_hat)

    b_omega = 307.5566 * math.pi / 180.0 # right acention in radians
    s_omega = 130.8460 * math.pi / 180.0

    # inclination, radians please
    i = 97.4404 * math.pi / 180.0

    A11 = math.cos(b_omega)*math.cos(s_omega) - math.sin(b_omega)*math.sin(s_omega)*math.cos(i)
    A12 = math.sin(b_omega)*math.cos(s_omega) + math.cos(b_omega)*math.sin(s_omega)*math.cos(i)
    
    A21 = -math.cos(b_omega)*math.sin(s_omega) - math.sin(b_omega)*math.cos(s_omega)*math.cos(i)
    A22 = -math.sin(b_omega)*math.sin(s_omega) + math.cos(b_omega)*math.cos(s_omega)*math.cos(i)
    
    A31 = math.sin(b_omega)*math.sin(i)
    A32 = -math.cos(b_omega)*math.sin(i)

  
    thing = numpy.array([[A11, A12],
                         [A21, A22],
                         [A31, A32]])

    #print(thing)
    r_position = (thing.dot(numpy.array([x_position, y_position])))
    r_velocity = (thing.dot(numpy.array([x_dot_position, y_dot_position])))

    print(r_position)
    print(r_velocity)
