#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import logging; logger = logging.getLogger("main")
FORMAT = '%(asctime)s - %(levelname)s: %(message)s'
logging.basicConfig(format=FORMAT, level=logging.WARNING)

import time

from flask import Flask, escape, url_for,render_template, g, request, redirect, jsonify, session
from werkzeug import secure_filename


import sys, os
from jinja2 import Environment, PackageLoader
import json

app = Flask(__name__, static_folder='static')

maze = []
width = 0
height = 0

STARTPOS=[1,1]
MAXLIFE = 10
MIN_TIME_BETWEEN_INTERACTIONS=0.2 #seconds

robots = {}

def store_map(rawmaze):

    global maze, width, height

    if is_map_loaded():
        logger.warning("Map already loaded. Ignoring it. Restart the backend if you want to update the map.")
        return


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

@app.route("/set/<name>/<x>/<y>")
def set_robot(name, x, y):
    logger.info("Placing robot %s to %s,%s" % (name,x,y))

    x = int(x)
    y=int(y)
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

@app.route("/")
def main():
    return render_template('map.html')

@app.route("/get_robots")
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



        complete_robots[k]["age"] = now - robots[k]["created"]

    return json.dumps(complete_robots)


def create_new_robot(name):
    logger.info("Placing new robot %s at start position" % name)
    robots[name] = {"pos": STARTPOS,
                    "created": time.time(),
                    "lastinteraction": 0,
                    "life": MAXLIFE
                    }

@app.route('/move/<name>/<direction>')
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

@app.route("/map", methods=['POST'])
def map():
    logger.info("Retrieving the map data...")

    
    store_map(json.loads([k for k in request.form.keys()][0]))
    return ""

@app.route("/life/<name>")
def life(name):
    return json.dumps(robots[name]["life"] if name in robots else 0)


