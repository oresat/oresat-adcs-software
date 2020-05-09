#!/usr/bin/env python3


from pydbus import SystemBus
import time


"""
A test client that will print all the dbus properties for the adcs.
"""


if __name__=="__main__":
    bus = SystemBus() # connect to bus
    dbus_client = bus.get("org.OreSat.ADCS")

    while True:
        print("Current State: {}".format(dbus_client.CurrentState))
        print("GPS Data: {}".format(dbus_client.GPS_Data))
        print("Star Tracker Data: {}".format(dbus_client.StarTrackerData))
        print("Gyro Data: {}".format(dbus_client.GyroData))
        print("Magnetometer Data: {}".format(dbus_client.MagnetometersData))
        print("Reaction Wheels Data: {}".format(dbus_client.ReactionWheelsData))
        print("Magnetorquer Data: {}".format(dbus_client.MagnetorquerData))
        print("")
        time.sleep(1)
