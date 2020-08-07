import threading, atexit
# from test_dbus import TestDBus
from pydbus import SystemBus
from gi.repository import GLib
from visualization_interface import VisualizationInterface
from time import sleep


def cleanup(vi):
    # clean up visualization resources
    print("Starting cleanup...")
    vi.destroy()
    print("All done.")


if __name__ == "__main__":

    vi = VisualizationInterface()
    vi.run()
    atexit.register(cleanup, vi)
