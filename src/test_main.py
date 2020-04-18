from pydbus import SystemBus
from gi.repository import GLib
from dbus_adaptor import DBus_Adaptor
from state_machine import StateMachine


DESTINATION = "org.OreSat.ADCS" # aka service name
INTERFACE_NAME = "org.OreSat.ADCS"
OBJECT_PATH = "/org/OreSat/ADCS"


if __name__ == "__main__":
    # make dbus adaptor
    state_machine = StateMachine()
    dbus_adaptor = DBus_Adaptor(state_machine)

    # set up bus
    bus = SystemBus()
    bus.publish(INTERFACE_NAME, dbus_adaptor)

    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt as e:
        loop.quit()
