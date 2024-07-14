

from oresat_adcs.system import simulator
import numpy as np


if __name__ == "__main__":
    temp = simulator.SimulatorDaemonInterface()

    # the maximum values for MTs are 60,000 and 300,000
    cmds = [np.array([0, 0, 300000]), np.array([0,0,0,0])]

    output = ""
    for blah in range(1000):
        out = temp.propagate(2, cmds)

        output += (",".join([str(component) for component in out[2]])) + "\n"

        if blah % 10 == 0:
            print("ok")

    print(output)
