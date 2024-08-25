
from oresat_adcs.functions import frame

if __name__ == "__main__":
    xyz = frame.geodetic_to_ECEF(54.34, 8.56, 516000)
    print(xyz)
