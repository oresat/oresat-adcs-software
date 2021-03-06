import threading, atexit, sys
# from test_dbus import TestDBus
from pydbus import SystemBus
from gi.repository import GLib
from visualization_interface import VisualizationInterface
from time import sleep

"""A test module for 42 Rx, to be used alongside test_dbus.py"""

def _cleanup(vi):
    # clean up visualization resources
    print("Starting cleanup...")
    vi.destroy()
    print("All done.")


if __name__ == "__main__":

    sys.path.append(".")
    vi = VisualizationInterface()
    vi.run()
    atexit.register(_cleanup, vi)
