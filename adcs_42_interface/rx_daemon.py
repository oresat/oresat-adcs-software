#!/usr/bin/env python3


import os, signal, sys
import psutil


class RxDaemon(object):
    """
    This class will initialize 42 Rx.
    """

    def __init__(self):
        """Initialize PID number for the forked process."""
        self._fork_pid = -1


    def run(self):
        """Launch 42 Rx as its own process."""
        try:
            os.chdir("./42Rx")
            os.system("./42 Rx &")
            return 0
        except OSError as err:
            return -1

    
    def quit(self):
        """Quit 42 Rx and clean up its resources."""
        processName = "42"
        success = False
        for proc in psutil.process_iter():
            try:
                print(proc.name())
                if processName == proc.name():
                    os.kill(proc.pid, signal.SIGTERM)
                    success = True
                    break
            except psutil.Error as err:
                pass

        if success:
            return 0
        else:
            return -1
