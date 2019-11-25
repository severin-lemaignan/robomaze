#! /usr/bin/env python

import sys
import select
import tty
import termios

import json
import requests

#URL = "https://robomaze.skadge.org/api?"
URL = "http://localhost/api?"

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)

if len(sys.argv) != 2:
    print("Usage: keyboard.py <name of robot>")
    sys.exit()

name = sys.argv[1]

try:
    tty.setcbreak(sys.stdin.fileno())

    v = 0.
    w = 0.

    while 1:
        if isData():
            c = sys.stdin.read(1)
            key = ord(c.decode('utf-8'))
            if c == '\x1b':         # x1b is ESC
                arrow = ord(sys.stdin.read(2)[1].decode('utf-8'))
                if arrow == 65: # up 
                    v += 0.1
                elif arrow == 66: # bottom
                    v -= 0.1
                elif arrow == 67: # right 
                    w += 0.1
                elif arrow == 68: # left 
                    w -= 0.1
                
                requests.get(URL + "cmd_vel=" + json.dumps([name, v,w]))

            else:
                print(key)



finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
