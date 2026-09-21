#!/usr/bin/env python3
"""ros2 bag play with keyboard controls that still exits cleanly on Ctrl-C.

rosbag2's keyboard handler blocks in read() on the terminal; on macOS SIGINT does not interrupt
that read, so the player hangs in its destructor until another key arrives. This wrapper forwards
Ctrl-C to the player and then keeps typing a newline into the terminal (TIOCSTI) until it exits.
"""
import fcntl
import os
import signal
import subprocess
import sys
import termios
import time


def main() -> int:
    child = subprocess.Popen(["ros2", "bag", "play", *sys.argv[1:]])
    interrupted = False

    def on_sigint(signum, frame):
        nonlocal interrupted
        interrupted = True
        child.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, on_sigint)
    while child.poll() is None:
        if interrupted:
            try:
                fd = os.open("/dev/tty", os.O_RDWR)
                fcntl.ioctl(fd, termios.TIOCSTI, b"\n")
                os.close(fd)
            except OSError:
                pass
        time.sleep(0.3)
    return child.returncode


if __name__ == "__main__":
    sys.exit(main())
