#!/usr/bin/env python3
"""A* pathfinding example: navigates a robot to the goal using incremental
maze discovery. The robot only knows obstacles revealed by the /api/move
endpoint and replans at every step.
"""

import sys
import time
import requests

URL = "http://localhost:5000"


def get_maze_info():
    """Fetch maze dimensions and goal from the server."""
    data = requests.get(f"{URL}/api/map").json()
    return data["width"], data["height"], tuple(data["goal"]) if data["goal"] else None


def astar(start, goal, maze, maze_size):

    cost_to = {start: 0}
    come_from = {}
    nodes_to_visit = [(start, 0)]

    while nodes_to_visit:
        node, _ = pop_best_node(nodes_to_visit)

        if node == goal:
            break

        for neighbour in neighbours(node, maze, maze_size):
            cost_to.setdefault(neighbour, float('inf'))
            if cost_to[node] + 1 < cost_to[neighbour]:
                cost_to[neighbour] = cost_to[node] + 1
                nodes_to_visit.append(
                    (neighbour, cost_to[neighbour] + heuristic(neighbour, goal)))
                come_from[neighbour] = node

    # Reconstruct path
    path = []
    node = goal
    while node in come_from:
        path = [node] + path
        node = come_from[node]
    return path


def heuristic(node, goal):
    return (node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2


def pop_best_node(nodes):
    """Pop the node with the lowest score.

    Note: this is a naive O(n) implementation. Using a heap (heapq) would
    be much faster -- can you improve it?
    """
    best_idx = 0
    best_score = float('inf')
    for idx in range(len(nodes)):
        if nodes[idx][1] < best_score:
            best_score = nodes[idx][1]
            best_idx = idx
    return nodes.pop(best_idx)


def is_free(node, maze, maze_size):
    x, y = node
    w, h = maze_size
    if x < 0 or y < 0 or x >= w or y >= h:
        return False
    return maze[y * w + x] == 0


def neighbours(node, maze, maze_size):
    x, y = node
    return [n for n in [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]
            if is_free(n, maze, maze_size)]


def update_maze(pos, obstacles, maze, maze_size):
    """Update the maze with newly discovered obstacles around pos."""
    x, y = pos

    # obstacles = [N, S, E, W]
    if obstacles[0]:
        maze[(y + 1) * maze_size[0] + x] = 1
    if obstacles[1]:
        maze[(y - 1) * maze_size[0] + x] = 1
    if obstacles[2]:
        maze[y * maze_size[0] + (x + 1)] = 1
    if obstacles[3]:
        maze[y * maze_size[0] + (x - 1)] = 1


def plan_next_move(pos, maze, maze_size, goal):
    """Run A* and return the direction to move next."""
    path = astar(pos, goal, maze, maze_size)
    if not path:
        return "E"  # fallback

    x, y = pos
    nx, ny = path[0]
    if nx - x == 1:
        return "E"
    elif nx - x == -1:
        return "W"
    elif ny - y == 1:
        return "N"
    elif ny - y == -1:
        return "S"

    return "E"


def get_robot_pos(name):
    """Fetch the actual robot position from the server."""
    robots = requests.get(f"{URL}/api/robots").json()
    if name in robots:
        return tuple(robots[name]["pos"])
    return None


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: astar.py <name of robot>")
        sys.exit(1)

    name = sys.argv[1]

    w, h, goal = get_maze_info()
    if goal is None:
        print("No goal set on the server! Start with: python -m robomaze.rest_server --goal")
        sys.exit(1)

    maze_size = (w, h)
    maze = [0] * (w * h)  # initially unknown — assume no obstacles

    # First move to create the robot and get initial position
    response = requests.get(f"{URL}/api/move/{name}/E").json()
    pos = get_robot_pos(name)
    if pos is None:
        print("Robot died on first move!")
        sys.exit(1)

    if response[0]:
        update_maze(pos, response[1], maze, maze_size)

    print(f"Navigating from {pos} to goal {goal} in a {w}x{h} maze...")

    while pos != goal:
        next_move = plan_next_move(pos, maze, maze_size, goal)

        response = requests.get(f"{URL}/api/move/{name}/{next_move}").json()
        success = response[0]

        # Always fetch actual position from server
        pos = get_robot_pos(name)
        if pos is None:
            print("Dead :-(")
            break

        if success:
            update_maze(pos, response[1], maze, maze_size)

        time.sleep(0.25)
    else:
        print(f"Reached the goal at {goal}!")
