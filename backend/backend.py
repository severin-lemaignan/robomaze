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

STARTPOS=[1,1]
MAXLIFE = 10
MIN_TIME_BETWEEN_INTERACTIONS=0.2 #seconds

robots = {
        "Wall-E": {"pos":[1, 1], "lastinteraction":0, "life":10}
        }

def store_map(mapdata):
    global maze, width, height

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

def get_obstacles(x,y):

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

def set_robot(name, x, y):
    logger.info("Placing robot %s to %s,%s" % (name,x,y))

    c,_,_,_,_ = get_obstacles(x,y)
    if c:
        logger.info("Can not place robot there!")
        return json.dumps(False)

    robots[name]["pos"] = [x,y]
    return json.dumps(True)

def get_robot(name):
    if name not in robots:
        return json.dumps([-1,-1])
    return json.dumps(robots[name]["pos"])

def get_robots():

    res = {}
    for name,v in robots.items():
        res[name] = v["pos"]

    return json.dumps(res)


def create_new_robot(name):
    logger.info("Placing new robot %s at start position" % name)
    robots[name] = {"pos": STARTPOS,
                    "lastinteraction": time.time(),
                    "life": MAXLIFE
                    }

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
            name,x,y = json.loads(options["set"][0])
            return set_robot(name,x,y)
        if "move" in options:
            name,direction = json.loads(options["move"][0])
            return move(name,direction)

        return ""


logger.info("ROBOMAZE backend. Starting to API...")
WSGIServer(app, bindAddress = ("127.0.0.1", 8081)).run()
logger.info("Bye bye.")
