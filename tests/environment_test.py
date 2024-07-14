
import numpy as np
from oresat_adcs.classes import jday
from oresat_adcs.configuration import environment

if __name__ == "__main__":
    my_env = environment.ReducedEnvironment()

    my_jday = jday.Clock(2024, 7, 7, 14, 0, 0)
    my_jclock = jday.JClock(2024, 7, 7, 14, 0, 0)

    pos = np.array([0,0,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([400000,0,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([0, 400000,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([0,0,400000])
    print(my_env.sun_vector(my_jday, pos))


    my_jday = jday.Clock(2024, 7, 28, 14, 0, 0)
    
    pos = np.array([0,0,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([400000,0,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([0, 400000,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([0,0,400000])
    print(my_env.sun_vector(my_jday, pos))

    '''
    [-0.27465406  0.88221983  0.38243079]
    [-0.27465653  0.88221919  0.38243051]
    [-0.27465471  0.88221924  0.3824317 ]
    [-0.27465434  0.88222074  0.38242851]
    [-0.58753792  0.74244135  0.32183853]
    [-0.58753967  0.74244018  0.32183802]
    [-0.58753908  0.74244015  0.32183917]
    [-0.58753842  0.74244199  0.32183613]
    '''



    pass
