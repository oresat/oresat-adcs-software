#!/usr/bin/env python3


from pydbus import SystemBus
from gi.repository import GLib
import time, random, datetime


#TODO gobals for now, fix later
bus = SystemBus() # connect to bus
dbus_client = bus.get("org.OreSat.ADCS")


def reaction_wheels_command_cb(*args):
    """
    Callback when recieving ReactionWheelsCommand signal.
    """

    print("Reaction Wheels Command Signal: ", args[4])


def magnetorquer_command_cb(*args):
    """
    Callback when recieving MagnetorquerCommand signal.
    """

    print("Magnetorquer Command Signal: ", args[4])


if __name__=="__main__":
    loop = GLib.MainLoop()

    # subscribe to the signals and regsister callbacks
    bus.subscribe(
            sender="org.OreSat.ADCS",
            iface="org.OreSat.ADCS",
            signal="ReactionWheelsCommand",
            signal_fired=reaction_wheels_command_cb,
            object="/org/OreSat/ADCCS")

    bus.subscribe(
            sender="org.OreSat.ADCS",
            iface="org.OreSat.ADCS",
            signal="MagnetorquerCommand",
            signal_fired=magnetorquer_command_cb,
            object="/org/OreSat/ADCS")

    try:
        dbus_client.CurrentState = 2
        loop.run()
    except KeyboardInterrupt as e:
        loop.quit()

