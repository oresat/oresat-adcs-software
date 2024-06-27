from pydbus import SystemBus
from gi.repository import GLib
import time
from threading import Thread
from syslog import syslog
from adcs_manager.adcs_lib.state_machine import *
from adcs_manager.dbus_server import DbusServer
from adcs_manager.adcs_lib import manager, simulator
import copy
import numpy as np

T_SLEEP  = 0.45
T_CTRL   = 0.05
DEBUG    = True

DESTINATION    =    "org.OreSat.ADCSManager"
INTERFACE_NAME =    "org.OreSat.ADCSManager"
OBJECT_PATH    =    "/org/OreSat/ADCSManager"
PID_FILE       =    "/run/oresat-linux-updater.pid"


class ADCSManager():
    """
    This class will handle initializing the program (making all class objects
    and starting all thread).
    """


    def __init__(self):
        """
        Make pid file / check if daemon is already running.
        """

        self._dbus_thread = Thread(target=self._dbus_server_thread)
        self._running     = False
        self._dbus_server = DbusServer()
        self._bus         = SystemBus()
        self._bus.publish(INTERFACE_NAME, self._dbus_server)
        self._dbus_loop = GLib.MainLoop()

        # adcs library interface
        self.sim = simulator.SimulatorDaemonInterface()
        self.man = manager_class.ManagerDaemonInterface(T_CTRL, T_CTRL, self.sim.model)

        # Is any of this commented out code still relevant?
        # Check for a pidfile to see if the daemon is already running
        #try:
        #    with open(PID_FILE,'r') as pf:

#                pid = int(pf.read().strip())
#        except IOError:
#            pid = None

#        if pid:
#            sys.stderr.write("pid file {0} already exist.\n".format(PID_FILE))
#            sys.exit(1)

#        try:
#            pid = os.fork()
#            if pid > 0:
#                # exit from parent
#                sys.exit(0)
#        except OSError as err:
#            sys.stderr.write('fork failed: {0}\n'.format(err))
#            sys.exit(1)

        # decouple from parent environment
#        os.chdir('/')
#        os.setsid()
#        os.umask(0)

        # redirect standard file descriptors
#        sys.stdout.flush()
#        sys.stderr.flush()
#        si = open(os.devnull, 'r')
#        so = open(os.devnull, 'a+')
#        se = open(os.devnull, 'a+')

#        os.dup2(si.fileno(), sys.stdin.fileno())
#        os.dup2(so.fileno(), sys.stdout.fileno())
#        os.dup2(se.fileno(), sys.stderr.fileno())

        # make pid file
#        pid = str(os.getpid())
#        with open(PID_FILE,'w+') as f:
#            f.write(pid + '\n')


    def __del__(self):
        """
        Decontructor to make sure the dbus thread is stopped and to remove
        the pid file.
        """

        # join the thread
        self._dbus_thread.join()

        # remove pid file
#        os.remove(PID_FILE)


    def _dbus_server_thread(self):
        """
        Nice function to hold the dbus thread.
        """

        self._dbus_loop.run()


    def run(self):
        """
        Main loop for adcs.
        """

        # for debugging
        if DEBUG:
            self.true_states   = []
            self.kalman_states = []


        # start dbus thread
        self._dbus_thread.start()

        self._dbus_server._sm.change_state(State.SLEEP)

        self._running = True
        counter       = 0 # for testing out periodic sensor measurements
        t0 = 0
        t1 = 0
        cmds          = [np.zeros(3), np.zeros(4)]
        sim_output    = None

        while(self._running):

            # sensor and control scheduling
            measure_this_cycle = counter == 0
            control_this_cycle = counter % (0.25 / T_CTRL) == 0
            raw_sensor_data    = self.sim.output(noisy=True) if measure_this_cycle else None
            counter            = (counter + 1) % (60 / T_CTRL)

            # copy all data from the dbus
            self._dbus_server._data_lock.acquire()
            dbus_data = copy.deepcopy(self._dbus_server._board_data)
            self._dbus_server._data_lock.release()

            # make adcs library calls
            current_state = self._dbus_server.CurrentMode
            next_mission  = self._dbus_server._sm.get_next_mission()

            if current_state is State.SLEEP or current_state is State.FAILED:
                man_output = self.man.propagate(T_SLEEP + T_CTRL, [current_state], raw_sensor_data)
                cmds       = man_output[:2] if control_this_cycle else cmds
                sim_output = self.sim.propagate(T_SLEEP + T_CTRL, cmds)
                if not DEBUG:
                    time.sleep(T_SLEEP)

            elif current_state is State.MANUAL: # low priority: give some interface for manual control
                man_output = self.man.propagate(T_CTRL, [current_state], raw_sensor_data)
                cmds       = man_output[:2] if control_this_cycle else cmds
                sim_output = self.sim.propagate(T_CTRL, cmds)

            else:
                if next_mission is None:
                    instrument = np.array([0, 0, -1])
                    target     = np.zeros(3)
                else:
                    instrument = self.man.filter.PosFilter.model.satellite.instruments[next_mission.instrument].boresight
                    target     = next_mission.target

                man_output = self.man.propagate(T_CTRL, [current_state, instrument, target], raw_sensor_data)
                cmds       = man_output[:2] if control_this_cycle else cmds
                sim_output = self.sim.propagate(T_CTRL, cmds[:2])

            [x, v, q, w, W, B, S]          = man_output[2]
            [mag_commands, rw_commands] = cmds

            # magnetorquer command
            self._dbus_server.MagnetorquerCommand(*mag_commands)
            self._dbus_server.NewMagCommands(*mag_commands)

            # reaction wheels command, call adcs lib functions here eventually
            self._dbus_server.ReactionWheelsCommand(*rw_commands)
            self._dbus_server.NewRWCommands(*rw_commands)

            # state transition function
            self._dbus_server._sm.transition(self.man, x, v, q, w, W, B, mag_commands, rw_commands)

            # send data to 42
            [sim_x, sim_v, sim_q, sim_w, sim_W, sim_B, sim_S] = sim_output
            self._dbus_server.VisualizationDataSignal(
                sim_S,              # sun-pointing unit vector
                sim_B,              # magnetic field vector
                self.man.rw_controller.MoI.dot(w),              # angular momentum
                sim_w,              # angular velocity
                np.array([*sim_q[1:], sim_q[0]]),              # spacecraft attitude
                sim_x,              # central orbit position
                sim_v               # central orbit velocity
            )

            if DEBUG:
                self.true_states.append(np.block([sim_x, sim_v, sim_q, sim_w, sim_W, sim_B, sim_S, self.man.filter.PosFilter.model.satellite.magnetorquers.power(mag_commands), np.sum([* np.abs(mag_commands)])]))
                self.kalman_states.append(np.block([x, v, q, w, W, B, 1 - np.dot(sim_q, q)])) # to look at error between state and estimate
                if self.man.filter.PosFilter.model.clock.absolute > 25000:
                    print('stopping simulation')
                    print('x:', x)
                    print('v:', v)
                    print('q:', q)
                    print('w:', w)
                    print('W:', W)
                    self.quit()

            if not DEBUG:
                time.sleep(T_CTRL) # so CPU is not at 100% all the time


    def quit(self):
        """
        Stop all thread.
        """
        if DEBUG:
            # for debugging
            np.savetxt('true_states.csv', self.true_states, delimiter=',', comments='',
                header='x_x,x_y,x_z,v_x,v_y,v_z,q0,q1,q2,q3,w_x,w_y,w_z,W1,W2,W3,W4,B_x,B_y,B_z,S_x,S_y,S_z,P,I,qd0,qd1,qd2,qd3')
            np.savetxt('kalman_states.csv', self.kalman_states, delimiter=',', comments='',
                header='x_x,x_y,x_z,v_x,v_y,v_z,q0,q1,q2,q3,w_x,w_y,w_z,W1,W2,W3,W4,B_x,B_y,B_z,S_x,S_y,S_z,roll,pitch,yaw')

        if DEBUG:
            # for debugging
            np.savetxt('true_states.csv', self.true_states, delimiter=',', comments='',
                header='x_x,x_y,x_z,v_x,v_y,v_z,q0,q1,q2,q3,w_x,w_y,w_z,W1,W2,W3,W4,B_x,B_y,B_z,S_x,S_y,S_z,P,I')
            np.savetxt('kalman_states.csv', self.kalman_states, delimiter=',', comments='',
                header='x_x,x_y,x_z,v_x,v_y,v_z,q0,q1,q2,q3,w_x,w_y,w_z,W1,W2,W3,W4,B_x,B_y,B_z,q_e_norm')

        self._running = False
        self._dbus_loop.quit()
