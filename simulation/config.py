from enum import Enum, IntEnum


class GuidanceMode(IntEnum):
    TARGET = 0
    NADIR = 1
    SUN = 2
    MIN_DRAG = 3
    MAX_DRAG = 4


class ControlMode(IntEnum):
    IDLE = 0
    RW_POINTING = 1
    MTB_POINTING = 2
    DETUMBLE = 3
    THERMAL_DETUMBLE = 4
    THERMAL_REORIENT = 5
    THERMAL_SPINUP = 6
    RW_SLOW_ROTATE = 91

class ControlActuators(IntEnum):
    NONE = 0
    RW_ONLY = 1
    MT_ONLY = 2
    RW_AND_MT = 3

class ControlPlan(IntEnum):
    # TODO: this is to have multi-stage algorithms
    IDLE = 0
    DETUMBLE = 1
    POINTING = 2
    # for example, the thermal control algorithm
    # first needs to detumble
    # then need or reorient
    # and finally needs to spin
    THERMAL = 3
    # TODO: overall I want to classify control types
    # 1) nothing
    # 2) only zero the velocity (detumble)
    # 3) only nonzero velocity (spin)
    # 3) only position control

class PointingReference(IntEnum):
    HELICAL = 0
    CIRRUS_FLUX = 1
    STAR_TRACKER = 2


class GainModeRW(Enum):
    STANDARD = 0
    TRANSIENT = 1
    FINE_POINTING = 2
