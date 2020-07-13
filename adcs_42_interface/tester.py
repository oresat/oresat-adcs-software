import threading
from test_dbus import TestDBus
from pydbus import SystemBus
from gi.repository import GLib
from visualization_interface import VisualizationInterface
from time import sleep


INTERFACE_NAME = "org.OreSat.ADCS"
dbusLoop = GLib.MainLoop()


def dbus_run():
    dbusLoop.run()


def main():
    """Driver to test the ADCS-42 interface."""
    # initialize dbus
    dbusServer = TestDBus()
    bus = SystemBus()
    bus.publish(INTERFACE_NAME, dbusServer)
    dbusThread = threading.Thread(target=dbus_run, name="dbus-thread")
    dbusThread.start()

    vi = VisualizationInterface()

    # send data to 42 for a minute
    for i in range(6000):
        vi.step()
        sleep(0.1)

    # cleanup
    vi.destroy()


if __name__ == "__main__":
    main()