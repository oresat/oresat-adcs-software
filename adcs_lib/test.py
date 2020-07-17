import numpy as np
import simulator, manager
# throwaway test file

sim = simulator.SimulatorDaemonInterface()
man = manager.ManagerDaemonInterface(sim.model)

cmds = man.propagate(0, [2, np.zeros(3), np.array([0,0,-1])], 0)
output = sim.propagate(0.5, cmds[:2])

print(cmds)
print()
print(output)
