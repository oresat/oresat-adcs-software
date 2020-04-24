from dbus_server import Dbus_Server


class MagnetorquerController:
    def __init__(self, dbus_adaptor_input):
        self._dbus_server = dbus_server_input

    def detumble(self, adcs_data):
        adcs_data_frame = self._dbus_server.get_adcs_data()

        # Do stuff

        return 0

    def point(self, adcs_data):
        adcs_data_frame = self._dbus_server.get_adcs_data()

        # Do stuff

        return 0
