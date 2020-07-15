import threading, atexit
from test_dbus import TestDBus
from pydbus import SystemBus
from gi.repository import GLib
from visualization_interface import VisualizationInterface
from time import sleep


def dbus_run():
    dbusLoop.run()


def cleanup(dbusLoop, vi, dbusThread):
    # clean up visualization resources
    print("Starting cleanup\n")
    dbusLoop.quit()
    print("Successfully quit dbus loop\n")
    vi.destroy()
    print("Successfully shut down 42\n")
    dbusThread.join()
    print("All done\n")


if __name__ == "__main__":

    # initialize dbus
    INTERFACE_NAME = "org.OreSat.ADCS"
    dbusServer = TestDBus()
    bus = SystemBus()
    bus.publish(INTERFACE_NAME, dbusServer)
    dbusLoop = GLib.MainLoop()
    
    vi = VisualizationInterface()
    dbusThread = threading.Thread(target = dbus_run, name="dbus-thread")
    dbusThread.start()

    atexit.register(cleanup, dbusLoop, vi, dbusThread)

    # send data to 42 for a minute
    for i in range(6000):
        vi.step()
        sleep(0.1)
