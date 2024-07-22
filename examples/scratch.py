
import numpy as np

if __name__ == "__main__":
    inclination = np.pi/3
    azimuth = np.pi/4

    thing = [np.array([np.sin(inclination)*np.cos(azimuth+rw_index*np.pi/2),
                       np.sin(inclination)*np.sin(azimuth+rw_index*np.pi/2),
                       np.cos(inclination)]) for rw_index in range(4)]

    print(thing)

    axes = np.array(thing).T

    print(axes)
