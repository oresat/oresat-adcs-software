import sys, os, getopt
from daemon import Daemon


def usage():
    message = """"
        usage:\n \
        python3 main.py      : to run as a process
        python3 main.py -d   : to run as a daemon
        python3 main.py -h   : this
        """

    print(message)


if __name__ == "__main__":
    adcs_daemon = Daemon()

    # deal with flags
    opts, args = getopt.getopt(sys.argv[1:], "h")
    for opt, arg in opts:
        if opt == "h":
            usage()
            exit(0)

    try:
        adcs_daemon.run()
    except KeyboardInterrupt as e:
        adcs_daemon.quit()

