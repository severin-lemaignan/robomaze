#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import logging; logger = logging.getLogger("main")
FORMAT = '%(asctime)s - %(levelname)s: %(message)s'
logging.basicConfig(format=FORMAT, level=logging.DEBUG)

import time
from itertools import imap

from cgi import escape
import sys, os
from flup.server.fcgi import WSGIServer
import urlparse
from jinja2 import Environment, PackageLoader
import json

maze = []
width = 0
height = 0


"""

Robot orientation:

0,0 +-------------------
    |  theta=90
    |  
    |  y
    |  ^    theta=45
    |  |
    |  R--> x   theta=0
    |

- x,y relative to map origin (top left corner)
- theta stored in radians
- one unit (along x/y) is equal to one tile
"""

STARTPOS=[1.,-1.,0.]
MAXLIFE = 10
MIN_TIME_BETWEEN_INTERACTIONS=0.2 #seconds

robots = {}

def store_map(mapdata):

    global maze, width, height

    if is_map_loaded():
        logger.warning("Map already loaded. Ignoring it. Restart the backend if you want to update the map.")
        return


    rawmaze = json.loads(mapdata)

    width = rawmaze["width"]
    height = rawmaze["height"]

    maze = [True if x in [399,431,463,492,493,494,495] else False for x in rawmaze["data"]] 

    for j in range(height):
        for i in range(width):
            idx = i + j * width
            if maze[idx]:
                sys.stdout.write('.')
            else:
                sys.stdout.write(' ')
        sys.stdout.write('\n')

    logger.info("Maze successfully loaded!")

def is_map_loaded():
    return width and height

def get_obstacles(x,y,theta):

    ######### TODO
    # disable obstacle detection for now
    return [False, False, False, False, False]

    # obstacle at centre, north, south, east, west?
    obstacles = [True, True, True, True, True]

    if x >= 0 and y >= 0 and x < width and y < height and maze[x + y * width]:
        obstacles[0] = False

    if x >= 0 and y-1 >= 0 and x < width and y-1 < height and maze[x + (y-1) * width]:
        obstacles[1] = False

    if x >= 0 and y+1 >= 0 and x < width and y+1 < height and maze[x + (y+1) * width]:
        obstacles[2] = False

    if x+1 >= 0 and y >= 0 and x+1 < width and y < height and maze[x+1 + y * width]:
        obstacles[3] = False

    if x-1 >= 0 and y >= 0 and x-1 < width and y < height and maze[x-1 + y * width]:
        obstacles[4] = False

    logger.info(str(obstacles))
    return obstacles

def set_robot(name, x, y,theta):
    logger.info("Placing robot %s to (x=%s,y=%s,theta=%s)" % (name,x,y,theta))

    c,_,_,_,_ = get_obstacles(x,y,theta)
    if c:
        logger.info("Can not place robot there!")
        return json.dumps(False)

    robots[name]["pos"] = [x,y,theta]
    return json.dumps(True)

def compute_robot_position(name):

    now = time.time()
    delta = now - robots[name].get("lastposupdate", now)
    robots[name]["lastposupdate"] = now

    v,w = robots[name]["cmd_vel"]
    dx = v * delta
    dy = 0
    dtheta = w * delta


    x,y,theta = robots[name]["pos"]
    robots[name]["pos"] = [x+dx, y+dy, theta+dtheta]

def get_robot(name):
    if name not in robots:
        return json.dumps([-1,-1])
    
    compute_robot_position(k)
    return json.dumps(robots[name]["pos"])


def get_robots():

    now = time.time()
    
    complete_robots = dict(robots)
    
    for k in list(robots.keys()):
        if robots[k]["life"] <= 0:
            logger.warning("Robot %s has no life left! killing it!" % k)
            del robots[k]
            del complete_robots[k]
            continue

        if now - robots[k]["lastinteraction"] > 60 * 10:
            logger.warning("Robot %s has not being used for 10 min. Removing it." % k)
            del robots[k]
            del complete_robots[k]
            continue


        compute_robot_position(k)

        complete_robots[k]["age"] = now - robots[k]["created"]

    return json.dumps(complete_robots)


def create_new_robot(name):
    logger.info("Placing new robot %s at start position" % name)
    robots[name] = {"pos": STARTPOS,
                    "cmd_vel": [0,0],
                    "created": time.time(),
                    "lastinteraction": 0,
                    "life": MAXLIFE
                    }

def cmd_vel_robot(name,v,w):
    if not is_map_loaded():
        logger.error("Map not loaded yet! Reload webpage.")
        return json.dumps([False,[]])

    if name not in robots:
        create_new_robot(name)

    logger.info("Setting (v,w) for robot %s to (%s,%s)" % (name,v,w))

    now = time.time()
    if now - robots[name]["lastinteraction"] < MIN_TIME_BETWEEN_INTERACTIONS:
        logger.error("Too many interactions with %s. Wait a bit." % name)
        return json.dumps([False,[]])

    robots[name]["lastinteraction"] = now
    compute_robot_position(name)
    robots[name]["cmd_vel"] = [v,w]



def move(name, direction):
    if not is_map_loaded():
        logger.error("Map not loaded yet! Reload webpage.")
        return json.dumps([False,[]])

    if name not in robots:
        create_new_robot(name)

    logger.info("Moving robot %s to %s" % (name,direction))

    now = time.time()
    if now - robots[name]["lastinteraction"] < MIN_TIME_BETWEEN_INTERACTIONS:
        logger.error("Too many interactions with %s. Wait a bit." % name)
        return json.dumps([False,[]])

    robots[name]["lastinteraction"] = now

    x,y = robots[name]["pos"]

    if direction == 'N':
        nx, ny = x, y-1
    if direction == 'S':
        nx, ny = x, y+1
    if direction == 'E':
        nx, ny = x+1, y
    if direction == 'W':
        nx, ny = x-1, y

    c,n,s,e,w = get_obstacles(nx,ny)

    if c:
        logger.info("...can not move there!")
        robots[name]["life"] -= 1
        return json.dumps([False,[]])

    else:
        robots[name]["pos"] = [nx,ny]
        return json.dumps([True,[n,s,e,w]])

def app(environ, start_response):

    logger.info("Incoming request!")
    start_response('200 OK', [('Content-Type', 'text/html')])

    path = environ["PATH_INFO"].decode("utf-8")


    if "map" in environ["QUERY_STRING"]:
        logger.info("Retrieving the map data...")

        # load the POST content
        try:
            request_body_size = int(environ.get('CONTENT_LENGTH', 0))
        except (ValueError):
            logger.warning("No map data!")
            request_body_size = 0

        request_body = environ['wsgi.input'].read(request_body_size)
        store_map(request_body)
        return ""
    else:
        options = urlparse.parse_qs(environ["QUERY_STRING"])

        if "get_robots" in environ["QUERY_STRING"]:
            return get_robots()
        if "set" in options:
            name,x,y,theta = json.loads(options["set"][0])
            return set_robot(name,x,y,theta)
        if "cmd_vel" in options:
            name,v,w = json.loads(options["cmd_vel"][0])
            return cmd_vel_robot(name,v,w)
        if "move" in options:
            name,direction = json.loads(options["move"][0])
            return move(name,direction)

        if "life" in options:
            name = json.loads(options["life"][0])
            return json.dumps(robots[name]["life"] if name in robots else 0)

        return ""


logger.info("ROBOMAZE backend. Starting the API...")
WSGIServer(app, bindAddress = ("127.0.0.1", 8081)).run()
logger.info("Bye bye.")
