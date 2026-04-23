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
    """Return the shortest path from start to goal using currently-known walls.

    The returned list excludes `start` and includes `goal`. An empty list is
    returned when no path to the goal exists under the current map.
    """
    # cost_to[n] = best known distance from start to n.
    cost_to = {start: 0}
    # come_from[n] = predecessor of n on the best path found so far.
    come_from = {}
    # Open set: nodes discovered but not yet expanded, paired with their
    # f-score = cost_to + heuristic-to-goal.
    nodes_to_visit = [(start, heuristic(start, goal))]

    while nodes_to_visit:
        node, _ = pop_best_node(nodes_to_visit)

        # With an admissible heuristic, the first time we pop the goal we
        # already hold the optimal path to it, so we can stop early.
        if node == goal:
            break

        for neighbour in neighbours(node, maze, maze_size):
            cost_to.setdefault(neighbour, float('inf'))
            if cost_to[node] + 1 < cost_to[neighbour]:
                cost_to[neighbour] = cost_to[node] + 1
                nodes_to_visit.append(
                    (neighbour, cost_to[neighbour] + heuristic(neighbour, goal)))
                come_from[neighbour] = node

    # Walk predecessors from goal back to start to reconstruct the path.
    path = []
    node = goal
    while node in come_from:
        path.append(node)
        node = come_from[node]
    path.reverse()
    return path


def heuristic(node, goal):
    """Estimated remaining cost from `node` to `goal`.

    On a 4-connected grid with unit step cost, the correct (admissible)
    heuristic is the Manhattan distance: the number of moves needed if
    there were no walls. A* is only guaranteed to return an optimal path
    when the heuristic never *overestimates* the true cost -- using
    something like squared Euclidean distance breaks that guarantee and
    produces visibly zigzagging paths.
    """
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])


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
    """True if `node` is inside the maze and not a known wall."""
    x, y = node
    w, h = maze_size
    if x < 0 or y < 0 or x >= w or y >= h:
        return False
    return maze[y * w + x] == 0


def neighbours(node, maze, maze_size):
    """Free 4-connected neighbours of `node`."""
    x, y = node
    return [n for n in [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]
            if is_free(n, maze, maze_size)]


def update_maze(pos, obstacles, maze, maze_size):
    """Record walls reported by the robot's sensors into our map.

    The server's coordinate convention (see robomaze/rest_server.py) is:
        N = y+1, S = y-1, E = x+1, W = x-1
    and `obstacles` arrives ordered as [N, S, E, W].
    """
    x, y = pos
    w = maze_size[0]

    if obstacles[0]: maze[(y + 1) * w + x] = 1  # wall to the north
    if obstacles[1]: maze[(y - 1) * w + x] = 1  # wall to the south
    if obstacles[2]: maze[y * w + (x + 1)] = 1  # wall to the east
    if obstacles[3]: maze[y * w + (x - 1)] = 1  # wall to the west


def plan_next_move(pos, maze, maze_size, goal):
    """Run A* and return the direction ('N'/'S'/'E'/'W') of the next step."""
    path = astar(pos, goal, maze, maze_size)
    if not path:
        return "E"  # fallback: no path known, try moving east and re-plan

    x, y = pos
    nx, ny = path[0]

    if nx == x + 1: return "E"
    if nx == x - 1: return "W"
    if ny == y + 1: return "N"
    if ny == y - 1: return "S"

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
