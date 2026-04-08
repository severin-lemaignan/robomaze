"""Flask REST API for robomaze.

Usage:
    python -m robomaze.rest_server [--random] [--seed N] [--width W] [--height H] [--goal]

Then open http://localhost:5000/live in a browser to see the maze,
and use the REST API to control robots:
    curl http://localhost:5000/api/move/WallE/E
"""

import argparse
import json
import os
import time

from flask import Flask, render_template, send_from_directory

from robomaze.maze import (
    Maze, FLOOR, BACKGROUND,
    TILESET_POSITIONS, TILESET_FLOOR_POSITIONS,
    TILESET_BACKGROUND_POSITIONS, TILESET_TILE_SIZE, TILESET_COLS,
)

# Paths
_dir = os.path.dirname(os.path.abspath(__file__))
_res_dir = os.path.join(_dir, '..', 'res')
_web_dir = os.path.join(_dir, '..', 'web')

app = Flask(__name__,
            template_folder=os.path.join(_web_dir, 'templates'),
            static_folder=os.path.join(_web_dir, 'static'))

# --- Robot state (tile-based, discrete movement) ---

MAXLIFE = 10
MIN_MOVE_INTERVAL = 0.2  # seconds
INACTIVITY_TIMEOUT = 600  # 10 minutes

robots = {}


class RestRobot:
    def __init__(self, name, x=1, y=1):
        self.name = name
        self.x = x
        self.y = y
        self.life = MAXLIFE
        self.created = time.time()
        self.lastinteraction = 0
        self.finished_time = None

    def get_obstacles(self):
        """Return [N, S, E, W] obstacle booleans for current position."""
        return [
            Maze.is_obstacle_raw(self.x, self.y + 1),
            Maze.is_obstacle_raw(self.x, self.y - 1),
            Maze.is_obstacle_raw(self.x + 1, self.y),
            Maze.is_obstacle_raw(self.x - 1, self.y),
        ]

    def move(self, direction):
        """Move one tile. Returns (success, obstacles_list)."""
        dx, dy = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}[direction]
        nx, ny = self.x + dx, self.y + dy

        if Maze.is_obstacle_raw(nx, ny):
            self.life -= 1
            return False, []

        self.x, self.y = nx, ny
        return True, self.get_obstacles()

    def to_dict(self):
        if self.finished_time is not None:
            age = self.finished_time - self.created
        else:
            age = time.time() - self.created
        return {
            'pos': [self.x, self.y],
            'created': self.created,
            'lastinteraction': self.lastinteraction,
            'life': self.life,
            'age': age,
            'finished': self.finished_time is not None,
        }


def _find_spawn_position():
    """Find a random open tile for spawning, away from the goal."""
    import random
    open_tiles = [
        (x, y)
        for y in range(Maze.height)
        for x in range(Maze.width)
        if not Maze.is_obstacle_raw(x, y)
    ]
    random.shuffle(open_tiles)

    goal = Maze.goal
    for tx, ty in open_tiles:
        if goal is not None:
            dist = ((tx - goal[0]) ** 2 + (ty - goal[1]) ** 2) ** 0.5
            if dist < 3:
                continue
        return tx, ty

    # Fallback
    return open_tiles[0] if open_tiles else (1, 1)


def _cleanup_robots():
    """Remove dead and timed-out robots."""
    now = time.time()
    to_remove = []
    for name, robot in robots.items():
        if robot.life <= 0:
            to_remove.append(name)
        elif robot.lastinteraction and now - robot.lastinteraction > INACTIVITY_TIMEOUT:
            to_remove.append(name)
    for name in to_remove:
        del robots[name]


# --- REST API ---

@app.route('/api/move/<name>/<direction>')
def api_move(name, direction):
    if direction not in ('N', 'S', 'E', 'W'):
        return json.dumps([False, []])

    if name not in robots:
        sx, sy = _find_spawn_position()
        robots[name] = RestRobot(name, sx, sy)

    robot = robots[name]

    now = time.time()
    if robot.lastinteraction and now - robot.lastinteraction < MIN_MOVE_INTERVAL:
        return json.dumps([False, []])

    robot.lastinteraction = now
    success, obstacles = robot.move(direction)

    # Check if robot reached the goal
    if success and robot.finished_time is None and Maze.goal is not None:
        if (robot.x, robot.y) == tuple(Maze.goal):
            robot.finished_time = time.time()

    if robot.life <= 0:
        del robots[name]

    return json.dumps([success, obstacles])


@app.route('/api/life/<name>')
def api_life(name):
    if name in robots:
        return json.dumps(robots[name].life)
    return json.dumps(0)


@app.route('/api/robots')
def api_robots():
    _cleanup_robots()
    return json.dumps({name: r.to_dict() for name, r in robots.items()})


@app.route('/api/map')
def api_map():
    return json.dumps({
        'width': Maze.width,
        'height': Maze.height,
        'data': Maze.data,
        'goal': Maze.goal,
    })


@app.route('/api/render_map')
def api_render_map():
    render_data = Maze.compute_render_map()
    return json.dumps({
        'width': Maze.width,
        'height': Maze.height,
        'tiles': render_data['tiles'],
        'ccv_overlays': render_data['ccv_overlays'],
        'tileset': {
            'tile_size': TILESET_TILE_SIZE,
            'cols': TILESET_COLS,
            'positions': {str(k): list(v) for k, v in TILESET_POSITIONS.items()},
            'floor_positions': [list(p) for p in TILESET_FLOOR_POSITIONS],
            'background_positions': [list(p) for p in TILESET_BACKGROUND_POSITIONS],
        },
    })


# --- Resource serving ---

@app.route('/res/<path:filename>')
def serve_res(filename):
    return send_from_directory(os.path.abspath(_res_dir), filename)


# --- Web visualization ---

@app.route('/')
@app.route('/live')
def live():
    return render_template('live.html')


# --- Entry point ---

def main():
    parser = argparse.ArgumentParser(description='Robomaze REST server')
    parser.add_argument('--random', action='store_true', help='Generate random maze')
    parser.add_argument('--seed', type=int, default=-1, help='Random seed (-1 for random)')
    parser.add_argument('--width', type=int, default=20, help='Maze width in tiles')
    parser.add_argument('--height', type=int, default=20, help='Maze height in tiles')
    parser.add_argument('--goal', action='store_true', help='Enable goal')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=5000, help='Port to listen on')
    parser.add_argument('--debug', action='store_true', help='Enable Flask debug mode')
    args = parser.parse_args()

    Maze.width = args.width
    Maze.height = args.height

    if args.random:
        seed = args.seed if args.seed >= 0 else None
        actual_seed = Maze.generate_random(seed)
        print(f'Generated random maze {Maze.width}x{Maze.height} with seed: {actual_seed}')
    else:
        print('Using default hardcoded maze.')

    if args.goal:
        goal_tile = Maze.set_goal_top_right_open()
        print(f'Goal enabled at tile: {goal_tile}')

    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == '__main__':
    main()
