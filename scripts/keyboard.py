#! /usr/bin/env python

import sys
import select
import tty
import termios

import json
import requests

URL = "https://robomaze.skadge.org/api?"

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)

if len(sys.argv) != 2:
    print("Usage: keyboard.py <name of robot>")
    sys.exit()

name = sys.argv[1]

try:
    tty.setcbreak(sys.stdin.fileno())

    while 1:
        if isData():
            c = sys.stdin.read(1)
            key = ord(c.decode('utf-8'))
            if c == '\x1b':         # x1b is ESC
                arrow = ord(sys.stdin.read(2)[1].decode('utf-8'))
                if arrow == 65: # up 
                    requests.get(URL + "move=" + json.dumps([name, "N"]))
                elif arrow == 66: # bottom
                    requests.get(URL + "move=" + json.dumps([name, "S"]))
                elif arrow == 67: # right 
                    requests.get(URL + "move=" + json.dumps([name, "E"]))
                elif arrow == 68: # left 
                    requests.get(URL + "move=" + json.dumps([name, "W"]))

            else:
                print(key)



finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
