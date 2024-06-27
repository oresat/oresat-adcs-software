


if __name__ == "__main__":
    
    import oresat_adcs
    print("dir(oresat_adcs): ", dir(oresat_adcs))

    import oresat_adcs.lib
    print("dir(oresat_adcs.lib): ", dir(oresat_adcs.lib))
    
    import oresat_adcs.generic
    print("dir(oresat_adcs.generic): ", dir(oresat_adcs.generic))
    

    from oresat_adcs import lib
    print("dir(lib): ", dir(lib))

    from oresat_adcs import generic
    print("dir(generic): ", dir(generic))
    
    from oresat_adcs import manager
    print("dir(manager): ", dir(manager))
