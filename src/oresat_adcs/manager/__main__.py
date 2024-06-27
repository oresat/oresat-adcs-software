import sys, os, getopt
from .manager import ADCSManager


def usage():
    message = """"
        usage:\n \
        python3 main.py      : to run as a process
        python3 main.py -d   : to run as a daemon
        python3 main.py -h   : this
        """

    print(message)


adcs_manager = ADCSManager()

# deal with flags
opts, args = getopt.getopt(sys.argv[1:], "h")
for opt, arg in opts:
    if opt == "h":
        usage()
        exit(0)

try:
    adcs_manager.run()
except KeyboardInterrupt as e:
    adcs_manager.quit()

