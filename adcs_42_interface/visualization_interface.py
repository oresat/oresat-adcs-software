#!/usr/bin/env python3


import datetime, socket
from pydbus import SystemBus
import time
from rx_daemon import RxDaemon


DBUS_INTERFACE_NAME = "org.OreSat.ADCS"
HOST = "localhost"
PORT = 7778


class VisualizationInterface:

    def __init__(self):
        """Initializes dbus connection, socket to 42 Rx, and simulation variables."""
        self._dbus = SystemBus().get(DBUS_INTERFACE_NAME)

        # 42 setup
        self._42_daemon = RxDaemon()
        self._42_daemon.run()

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.bind((HOST, PORT))
        self._socket.listen()
        self._42_socket, addr = self._socket.accept()
        
        # Simulation constants
        self._SC_POSR = (0.0, 0.0, 0.0)
        self._SC_VELR = (0.0, 0.0, 0.0)

        # Variables to be read in from dbus
        self._sun_pointing_unit_vector = (0.0, 0.0, 0.0)
        self._magnetic_field_vector = (0.0, 0.0, 0.0)
        self._angular_momentum = (0.0, 0.0, 0.0)
        self._angular_velocity = (0.0, 0.0, 0.0)
        self._spacecraft_attitude = (0.0, 0.0, 0.0, 0.0)
        self._central_orbit_position = (0.0, 0.0, 0.0)
        self._central_orbit_velocity = (0.0, 0.0, 0.0)
        self._earth_posh = (0.0, 0.0, 0.0)
        self._earth_orbit_position = (0.0, 0.0, 0.0)
        self._earth_orbit_velocity = (0.0, 0.0, 0.0)

    
    def _get_data_from_dbus(self):
        """Fetches simulation data from the dbus."""
        self._sun_pointing_unit_vector = self._dbus.SunPointingUnitVector42
        self._magnetic_field_vector = self._dbus.MagneticFieldVector42
        self._angular_momentum = self._dbus.AngularMomentum42
        self._angular_velocity = self._dbus.AngularVelocity42
        self._spacecraft_attitude = self._dbus.SpacecraftAttitude42
        self._central_orbit_position = self._dbus.CentralOrbitPosition42
        self._central_orbit_velocity = self._dbus.CentralOrbitVelocity42
        self._earth_posh = self._dbus.EarthPosH42
        self._earth_orbit_position = self._dbus.EarthOrbitPosition42
        self._earth_orbit_velocity = self._dbus.EarthOrbitVelocity42

    
    def _send_data_to_socket(self):
        """Sends simulation data to 42 Rx."""
        join_tpl = lambda tpl : " ".join([str(x) for x in tpl])

        time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        msg = (f"TIME {time}\n" +
        f"SC[0].PosR = {join_tpl(self._SC_POSR)}\n" + 
        f"SC[0].VelR = {join_tpl(self._SC_VELR)}\n" +
        f"SC[0].svb = {join_tpl(self._sun_pointing_unit_vector)}\n" +
        f"SC[0].bvb = {join_tpl(self._magnetic_field_vector)}\n" +
        f"SC[0].Hvb = {join_tpl(self._angular_momentum)}\n" +
        f"SC[0].AC.ParmLoadEnabled = 0\n" +
        f"SC[0].AC.ParmDumpEnabled = 0\n" + # not sure what these two commands do, hard-coded for now
        f"SC[0].B[0].wn = {join_tpl(self._angular_velocity)}\n" +
        f"SC[0].B[0].qn = {join_tpl(self._spacecraft_attitude)}\n" +
        f"Orb[0].PosN = {join_tpl(self._central_orbit_position)}\n" +
        f"Orb[0].VelN = {join_tpl(self._central_orbit_velocity)}\n" +
        f"World[3].PosH = {join_tpl(self._earth_posh)}\n" +
        f"World[3].eph.PosN = {join_tpl(self._earth_orbit_position)}\n" +
        f"World[3].eph.VelN = {join_tpl(self._earth_orbit_velocity)}\n")

        self._42_socket.send(msg.encode())


    def step(self):
        """Run one iteration/frame of the simulation."""
        self._get_data_from_dbus()
        self._send_data_to_socket()


    def destroy(self):
        """Stop the simulation by closing the connection with 42 Rx."""
        self._socket.shutdown(socket.SHUT_RDWR)
        self._socket.close()
        self._42_daemon.quit()
        del self
    