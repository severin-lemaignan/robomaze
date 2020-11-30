#! /usr/bin/env python

import sys, select, tty, termios,time

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)


import rospy
from geometry_msgs.msg import Twist



try:
    tty.setcbreak(sys.stdin.fileno())

    v = 0.
    w = 0.

    while 1:
        if isData():
            c = sys.stdin.read(1)
            key = ord(c)
            if c == '\x1b':         # x1b is ESC
                arrow = ord(sys.stdin.read(2)[1])
                if arrow == 65: # up 
                    v += 0.1
                elif arrow == 66: # bottom
                    v -= 0.1
                elif arrow == 67: # right 
                    w -= 0.1
                elif arrow == 68: # left 
                    w += 0.1
                else: # attention: due to the read(2), need to press esc 3 times to get there!
                    print("Exiting")
                    break
                print("v=%.1f, w=%.1f" % (v,w))

            else:
                if c == "q":
                    print("Exiting")
                    break

        time.sleep(0.01)

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
