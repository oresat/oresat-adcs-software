"""OreSat ADCS Manager"""

# keep this insync with debian/changelog
MAJOR = 0
MINOR = 1
PATCH = 0

APP_NAME = "oresat-adcs-manager"
APP_DESCRIPTION = "A daemon that manages OreSat's ADCS subsystem"
APP_VERSION = "{}.{}.{}".format(MAJOR, MINOR, PATCH)
APP_AUTHOR = "Ryan Medick"
APP_EMAIL = "rmedick@pdx.edu"
APP_URL = "https://github.com/oresat/oresat-adcs-manager"
APP_LICENSE = "GPL-3.0"

PID_FILE = "/run/oresat-adcs-managerd.pid"
