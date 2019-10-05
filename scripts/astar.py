#! /usr/bin/env python

import sys

import math
import time
import json
import requests

URL = "https://robomaze.skadge.org/api?"


MAZE_SIZE = (100, 100)
END_GOAL = (98,98)
pos = (1,1)

maze = [0,] * MAZE_SIZE[0] * MAZE_SIZE[1] # initially, our maze is empty

def astar(start, goal):

  cost_to = {} # maps nodes to distance to 'start_node'
  cost_to[start] = 0
  come_from = {} # needed to reconstruct shortest path

  nodes_to_visit = [(start,0)] # (node, 'total' cost)

  while nodes_to_visit:
    node, _ = pop_best_node(nodes_to_visit)

    for neighbour in neighbours(node):
      cost_to.setdefault(neighbour, float('inf'))
      if cost_to[node] + 1  < cost_to[neighbour]: # new shorter path to v!
        cost_to[neighbour] = cost_to[node] + 1
        nodes_to_visit.append((neighbour, cost_to[neighbour] + heuristic(neighbour, goal)))
        come_from[neighbour] = node

  # finally, reconstruct the path
  path = []
  node = goal
  while node in come_from:
    path = [node] + path # append at the front of our path
    node = come_from[node]
  return path

def heuristic(node, goal):
    return (node[0] - goal[0])**2 + (node[1] - goal[1])**2

def pop_best_node(nodes):
    '''
    !! Highly ineffective implementation, and major source of slow down.
    Can you do better?
    '''
    best_node_score = float('inf')
    best_node_idx = 0

    for idx in range(len(nodes)):
        if nodes[idx][1] < best_node_score:
            best_node_score = nodes[idx][1]
            best_node_idx = idx

    return nodes.pop(best_node_idx)


def is_free(node):
    x, y = node
    if x < 0 or y < 0 or x >= MAZE_SIZE[0] or y >= MAZE_SIZE[1]:
        return False

    return maze[y * MAZE_SIZE[0] + x] == 0

def neighbours(node):
    x,y = node
    return [n for n in [(x, y-1), (x, y+1), (x-1, y), (x+1, y)] if is_free(n)]

def get_next_move(obstacles):
    global pos, maze

    # update the maze

    x, y = pos
    if obstacles[0]:
        maze[(y-1) * MAZE_SIZE[0] + x] = 1
    if obstacles[1]:
        maze[(y+1) * MAZE_SIZE[0] + x] = 1
    if obstacles[2]:
        maze[y * MAZE_SIZE[0] + x+1] = 1
    if obstacles[3]:
        maze[y * MAZE_SIZE[0] + x-1] = 1

    # run the A*
    path = astar(pos, END_GOAL)

    direction = ""

    # update pos and return best move
    if path[0][0] - x == 1:
        x += 1
        direction =  "E"
    elif path[0][0] - x == -1:
        x -= 1
        direction =  "W"
    elif path[0][1] - y == 1:
        y += 1
        direction =  "S"
    elif path[0][1] - y == -1:
        y -= 1
        direction =  "N"
    else:
        raise Exception("Impossible move!")

    pos = (x,y)
    return direction

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: random.py <name of robot>")
        sys.exit()

    name = sys.argv[1]

    next_move = "E"

    while 1:
        response = requests.get(URL + "move=" + json.dumps([name, next_move]))

        obstacles = json.loads(response.text)[1]

        next_move = get_next_move(obstacles)

        time.sleep(0.25)



