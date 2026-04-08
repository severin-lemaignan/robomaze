#!/usr/bin/env python3
"""Keyboard-controlled robot: use arrow keys to move."""

import sys
import select
import tty
import termios

import requests

URL = "http://localhost:5000"


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


if len(sys.argv) != 2:
    print("Usage: keyboard.py <name of robot>")
    sys.exit(1)

name = sys.argv[1]

old_settings = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())

    while True:
        if is_data():
            c = sys.stdin.read(1)
            if c == '\x1b':  # ESC — start of arrow key sequence
                arrow = ord(sys.stdin.read(2)[1])
                if arrow == 65:    # up
                    requests.get(f"{URL}/api/move/{name}/N")
                elif arrow == 66:  # down
                    requests.get(f"{URL}/api/move/{name}/S")
                elif arrow == 67:  # right
                    requests.get(f"{URL}/api/move/{name}/E")
                elif arrow == 68:  # left
                    requests.get(f"{URL}/api/move/{name}/W")
            elif c == 'q':
                break

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
