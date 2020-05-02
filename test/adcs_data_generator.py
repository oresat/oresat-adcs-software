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

    #NOTE args[4] is the data sent

    print("Reaction Wheels Command Signal: ", args[4])

    # NOTE replace random number generator place holder

    # update the gps data
    dbus_client.GPS_Data = (
            ( # postion x,y,z
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0)
            ),
            ( # velocity x,y,z
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0)
            ),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )

    # update star tracker data
    dbus_client.StarTrackerData = (
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            )


    # update magnetometer data
    dbus_client.MagnetometersData = [
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            )
            ]

    # update reaction wheels data
    dbus_client.ReactionWheelsData = [
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            ]

    # update magnetorquer data
    dbus_client.MagnetorquerData = [
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            ]



def magnetorquer_command_cb(*args):
    """
    Callback when recieving MagnetorquerCommand signal.
    """

    #NOTE args[4] is the data sent

    print("MagnetorquerCommand Signal: ", args[4])

    # NOTE replace random number generator place holder

    # update the gps data
    dbus_client.GPS_Data = (
            ( # postion x,y,z
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0)
            ),
            ( # velocity x,y,z
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0)
            ),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )

    # update star tracker data
    dbus_client.StarTrackerData = (
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0),
            random.uniform(0.0, 100.0),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            )


    # update magnetometer data
    dbus_client.MagnetometersData = [
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            )
            ]

    # update reaction wheels data
    dbus_client.ReactionWheelsData = [
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            ]

    # update magnetorquer data
    dbus_client.MagnetorquerData = [
            (
            random.randint(0, 100),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            ),
            ]


if __name__=="__main__":
    loop = GLib.MainLoop()

    # subscribe to the signals and regsister callbacks
    bus.subscribe(
            sender="org.OreSat.ADCS",
            iface="org.OreSat.ADCS",
            signal="MagnetorquerCommand",
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
