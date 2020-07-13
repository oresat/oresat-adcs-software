

import threading, datetime


class TestDBus(object):
    dbus = """
    <node>
        <interface name="org.OreSat.ADCS">
            <property name="SunPointingUnitVector42" type="(ddd)" access="read"/>
            <property name="MagneticFieldVector42" type="(ddd)" access="read"/>
            <property name="AngularMomentum42" type="(ddd)" access="read"/>
            <property name="AngularVelocity42" type="(ddd)" access="read"/>
            <property name="SpacecraftAttitude42" type="(dddd)" access="read"/>
            <property name="CentralOrbitPosition42" type="(ddd)" access="read"/>
            <property name="CentralOrbitVelocity42" type="(ddd)" access="read"/>
            <property name="EarthPosH42" type="(ddd)" access="read"/>
            <property name="EarthOrbitPosition42" type="(ddd)" access="read"/>
            <property name="EarthOrbitVelocity42" type="(ddd)" access="read"/>
        </interface>
    </node>
    """ # this wont work in __init__()


    def __init__(self):
        """
        Contructor.
        """

        self._lock = threading.Lock()

        # initize tuples. must match xml type field
        time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._sun_pointing_unit_vector = (1.0, 2.0, 3.0)
        self._magnetic_field_vector = (4.0, 5.0, 6.0)
        self._angular_momentum = (7.0, 8.0, 9.0)
        self._angular_velocity = (10.0, 11.0, 12.0)
        self._spacecraft_attitude = (13.0, 14.0, 15.0, 16.0)
        self._central_orbit_position = (17.0, 18.0, 19.0)
        self._central_orbit_velocity = (20.0, 21.0, 22.0)
        self._earth_posh = (23.0, 24.0, 25.0)
        self._earth_orbit_position = (26.0, 27.0, 28.0)
        self._earth_orbit_velocity = (29.0, 30.0, 31.0)        


    # ------------------------------------------------------------------------
    # 42


    @property
    def SunPointingUnitVector42(self):
        """
        Getter for the sun-pointing unit vector used by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._sun_pointing_unit_vector
        self._lock.release()

        return temp


    @property
    def MagneticFieldVector42(self):
        """
        Getter for the magnetic field vector used by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._magnetic_field_vector
        self._lock.release()

        return temp


    @property
    def AngularMomentum42(self):
        """
        Getter for angular momentum by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._angular_momentum
        self._lock.release()

        return temp


    @property
    def AngularVelocity42(self):
        """
        Getter for angular velocity used by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._angular_velocity
        self._lock.release()

        return temp


    @property
    def SpacecraftAttitude42(self):
        """
        Getter for spacecraft attitude used by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._spacecraft_attitude
        self._lock.release()

        return temp


    @property
    def CentralOrbitPosition42(self):
        """
        Getter for central orbit position used by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._central_orbit_position
        self._lock.release()

        return temp


    @property
    def CentralOrbitVelocity42(self):
        """
        Getter for central orbit velocity used by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._central_orbit_velocity
        self._lock.release()

        return temp


    @property
    def EarthPosH42(self):
        """
        Getter for the H-frame position of Earth used by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._earth_posh
        self._lock.release()

        return temp


    @property
    def EarthOrbitPosition42(self):
        """
        Getter for Earth's orbit position used by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._earth_orbit_position
        self._lock.release()

        return temp


    @property
    def EarthOrbitVelocity42(self):
        """
        Getter for Earth's orbit velocity used by 42.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._earth_orbit_velocity
        self._lock.release()

        return temp
