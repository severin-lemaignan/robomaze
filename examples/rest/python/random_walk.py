#!/usr/bin/env python3
"""Random walk example: moves a robot in random directions."""

import sys
import random
import time
import requests

URL = "http://localhost:5000"

if len(sys.argv) != 2:
    print("Usage: random_walk.py <name of robot>")
    sys.exit(1)

name = sys.argv[1]
directions = ["N", "S", "E", "W"]

starttime = time.time()

try:
    while True:
        result = requests.get(f"{URL}/api/move/{name}/{random.choice(directions)}").json()
        print(result)

        life = requests.get(f"{URL}/api/life/{name}").json()
        if life == 0:
            print("Dead :-(")
            break

        time.sleep(0.25)

finally:
    print("Total elapsed time: %d seconds" % (time.time() - starttime))
