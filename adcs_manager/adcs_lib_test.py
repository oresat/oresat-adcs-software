import numpy as np
from adcs_manager.adcs_lib import simulator, manager
# throwaway test file

sim = simulator.SimulatorDaemonInterface()
man = manager.ManagerDaemonInterface(sim.model)

cmds = man.propagate(0.5, [2, np.zeros(3), np.array([0,0,-1])], 0)
output = sim.propagate(0.5, cmds[:2])

print(cmds)
print()
print(output)
print(np.linalg.norm(output[-1]))
