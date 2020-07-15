#!/usr/bin/env python3


import os, signal, sys


PID_FILE = "/run/42rx.pid"


class RxDaemon(object):
    """
    This class will initialize 42 Rx.
    """

    def __init__(self):
        """Initialize PID number for 42 Rx."""
        self._42pid = -1


    def run(self):
        """Launch 42 Rx as its own process."""
        try:
            self._42pid = os.fork()
        except OSError as err:
            sys.stderr.write('fork failed: {0}\n'.format(err))
            sys.exit(1)

        if self._42pid == 0:
            os.chdir("./42Rx")
            os.system("./42 Rx &")

    
    def quit(self):
        """Quit 42 Rx and clean up its resources."""
        # check that 42 is actually running
        if self._42pid < 0:
            sys.stderr.write("Couldn't detect a current instance of 42, aborting\n")
            return

        os.kill(self._42pid, signal.SIGTERM)
        if os.path.exists(PID_FILE):
            os.remove(PID_FILE)
        else:
            sys.stderr.write("Couldn't remove pid file for 42\n")
