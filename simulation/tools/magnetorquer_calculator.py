import numpy as np

def calculate_rod_permeability(
    rod_diameter,
    rod_length,
    rod_mu
) -> float:
    """
    Parameters
    ----------
    rod_diameter
        Radius of the permaolloy bar [m]
    rod_length
        Length of the magnetorquer rod [m]
    rod_mu
        Relative magnetic permeability of permalloy [?]
    """

    # rod ratio changed to be based on rod diameter instead of rod radius
    # divide diameter by 2 if this this supposed to be radius
    rod_ratio = rod_length / (rod_diameter) 
    print(f"Rod ratio: {rod_ratio}")
    S_mag = (4 * (np.log(rod_ratio) - 1)) / ((rod_ratio) ** 2 - 4 * np.log(rod_ratio))
    print(f"S Mag: {S_mag}")
    K_rod = 1 + (rod_mu - 1) / (1 + (rod_mu - 1) * S_mag)
    print(f"K Rod: {K_rod}")
    return K_rod


if __name__ == "__main__":

    # current = dipole / (windings * area * permeability)
    # therefore:
    # dipole = current * windings * area * permeability

    # number of turns
    rod_windings = 1700
    # outer diameter
    OD = 10.5e-3
    # inner diameter of windings
    ID = 6.35e-3
    # length of rod
    rod_length = 71e-3

    # relative permeability of the core material
    rod_mu = 100000  

    # This calculation for rod area is not what it appears to be
    # also make sure this is fixed in the function
    rod_area = np.pi * ((OD + ID) / 2) ** 2




    # initial K_rod value from C3: 58.81569652055702
    k_rod = calculate_rod_permeability(
        rod_diameter = ID,
        rod_length = rod_length,
        rod_mu = rod_mu
    )

    factor = k_rod * rod_windings * rod_area

    print(factor)


    rod_area = np.pi * ((ID) / 2) ** 2


    factor = k_rod * rod_windings * rod_area

    print(factor)


    rod_windings = 1700
    OD = 10.5e-3  # outer diameter of windings
    ID = 6.35e-3  # inner diameter of windings
    rod_area = np.pi * ((OD + ID) / 2) ** 2
    rod_length = 71e-3
    # [m] radius of just the core, used to determine the magnetic permeability of permalloy rod
    rod_radius = 6.35e-3 / 2
    rod_mu = 100000  # relative permeability of the core material
    rod_ratio = rod_length / rod_radius
    S_mag = (4 * (np.log(rod_ratio) - 1)) / ((rod_ratio) ** 2 - 4 * np.log(rod_ratio))
    K_rod = 1 + (rod_mu - 1) / (1 + (rod_mu - 1) * S_mag)  # magnetic permeability

    ring_windings = 505
    ring_area = 0.088**2 - (2 * ((0.0845 - 0.0604) / 2) ** 2)
    K_ring = 1  # air-core magnetorquer has magnetic permeability of 1

    # TODO: at some point, the current saturation should be
    # enforced via a dipole software saturation
    # [Equation] amps = dipole / (permeability * windings * area) = dipole * mag_constants

    mag_constants = 1e-6 * np.array(
        [
            1 / (K_rod * rod_windings * rod_area),
            1 / (K_rod * rod_windings * rod_area),
            1 / (K_ring * ring_windings * ring_area),
        ]
    )  # constants for each magnetorquer axis used to convert desired torques to current [uA]

    print(f"K Rod: {K_rod}")
    print(f"K Ring: {K_ring}")
    print(f"rod_area: {rod_area}")
    print(f"ring_area: {ring_area}")
