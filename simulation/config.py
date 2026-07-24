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


class PointingReference(IntEnum):
    HELICAL = 0
    CIRRUS_FLUX = 1
    STAR_TRACKER = 2


class GainModeRW(Enum):
    STANDARD = 0
    TRANSIENT = 1
    FINE_POINTING = 2
