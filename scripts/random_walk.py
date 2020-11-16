#! /usr/bin/env python

import sys

import random
import time
import json
import requests

REST_ENDPOINT = "https://robomaze.skadge.org/"

if len(sys.argv) != 2:
    print("Usage: random_walk.py <name of robot>")
    sys.exit()

name = sys.argv[1]

next_directions = ["N", "S", "E", "W"]


starttime = time.time()

try:

    while 1:

        result = requests.get(REST_ENDPOINT + "/move/" + name + "/" + random.choice(next_directions))
        print(json.loads(result.text))

        life = requests.get(REST_ENDPOINT + "/life/" + name)
        if json.loads(life.text) == 0:
            print("Dead :-(")
            break

        time.sleep(0.25)

finally:
    print("Total elapsed time: %d seconds" %(time.time() - starttime))
