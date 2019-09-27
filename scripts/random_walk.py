#! /usr/bin/env python

import sys

import random
import time
import json
import requests

URL = "https://robomaze.skadge.org/api?"


if len(sys.argv) != 2:
    print("Usage: random.py <name of robot>")
    sys.exit()

name = sys.argv[1]

next_directions = ["N", "S", "E", "W"]


starttime = time.time()

try:
    while 1:

        response = requests.get(URL + "move=" + json.dumps([name, random.choice(next_directions)]))
        obstacles = json.loads(response.text)[1]
        next_directions = [x for x in ["N", "S", "E", "W"] if not obstacles[["N", "S", "E", "W"].index(x)]]
        time.sleep(0.25)
finally:
    print("Total elapsed time: %d seconds" %(time.time() - starttime))
