import random
from collections import deque

import cv2
import numpy as np

# Tile type constants
FLOOR = 1
TOP = 2
BOTTOM = 3
LEFT = 4
RIGHT = 5
TOP_LEFT_CVX = 6
TOP_LEFT_CCV = 7
TOP_RIGHT_CVX = 8
TOP_RIGHT_CCV = 9
BOTTOM_LEFT_CVX = 10
BOTTOM_LEFT_CCV = 11
BOTTOM_RIGHT_CVX = 12
BOTTOM_RIGHT_CCV = 13
BOTTOM_TOP_RIGHT = 14
BOTTOM_TOP_LEFT = 15
BOTTOM_LEFT_RIGHT = 16
TOP_LEFT_RIGHT = 17
BOTTOM_TOP = 18
LEFT_RIGHT = 19

# Module-level globals populated by init_assets()
TILES = {}
ROBOT = None
ROBOT_ALPHA = None
ROBOT_RGB = None


def init_assets(res_root):
    """Load all tile images and robot sprite. Must be called before using Maze.

    Args:
        res_root: path to the res/ directory (with trailing slash)
    """
    global TILES, ROBOT, ROBOT_ALPHA, ROBOT_RGB

    ROBOT = cv2.imread(res_root + "walle_top.png", cv2.IMREAD_UNCHANGED)
    ROBOT_ALPHA = ROBOT[:, :, 3]
    ROBOT_RGB = ROBOT[:, :, :3]

    TILES.update({
        FLOOR: [
            cv2.imread(res_root + "floor2.jpg"),
            cv2.imread(res_root + "floor.jpg"),
            cv2.imread(res_root + "floor.jpg"),
            cv2.imread(res_root + "floor.jpg"),
        ],
        TOP: cv2.imread(res_root + "top.jpg"),
        BOTTOM: cv2.imread(res_root + "bottom.jpg"),
        LEFT: cv2.imread(res_root + "left.jpg"),
        RIGHT: cv2.imread(res_root + "right.jpg"),
        TOP_LEFT_CVX: cv2.imread(res_root + "top_left_cvx.jpg"),
        TOP_LEFT_CCV: cv2.imread(res_root + "top_left_ccv.jpg"),
        TOP_RIGHT_CVX: cv2.imread(res_root + "top_right_cvx.jpg"),
        TOP_RIGHT_CCV: cv2.imread(res_root + "top_right_ccv.jpg"),
        BOTTOM_LEFT_CVX: cv2.imread(res_root + "bottom_left_cvx.jpg"),
        BOTTOM_LEFT_CCV: cv2.imread(res_root + "bottom_left_ccv.jpg"),
        BOTTOM_RIGHT_CVX: cv2.imread(res_root + "bottom_right_cvx.jpg"),
        BOTTOM_RIGHT_CCV: cv2.imread(res_root + "bottom_right_ccv.jpg"),
        BOTTOM_TOP_RIGHT: cv2.imread(res_root + "bottom_top_right.jpg"),
        BOTTOM_TOP_LEFT: cv2.imread(res_root + "bottom_top_left.jpg"),
        BOTTOM_LEFT_RIGHT: cv2.imread(res_root + "bottom_left_right.jpg"),
        TOP_LEFT_RIGHT: cv2.imread(res_root + "top_left_right.jpg"),
        BOTTOM_TOP: cv2.imread(res_root + "bottom_top.jpg"),
        LEFT_RIGHT: cv2.imread(res_root + "left_right.jpg"),
    })

    # Resize tiles to match the pixel grid
    tile_px = Maze.TILESIZE * Maze.M2PX
    for k, v in TILES.items():
        if isinstance(v, list):
            TILES[k] = [cv2.resize(t, (tile_px, tile_px)) for t in v]
        else:
            TILES[k] = cv2.resize(v, (tile_px, tile_px))

    # Resize robot sprite
    robot_px = int(2 * Robot_RADIUS * Maze.M2PX)
    ROBOT = cv2.resize(ROBOT, (robot_px, robot_px))


# Robot radius constant (needed at module level for asset sizing)
# Must match Robot.RADIUS in robot.py
Robot_RADIUS = 0.25 * 5  # 0.25 * TILESIZE


class Maze:
    height = 20
    width = 20
    TILESIZE = 5  # meters
    M2PX = 7  # pixels per meter (TILESIZE * M2PX = tile size in pixels)

    # Goal position in tile coordinates (None = no goal)
    goal = None  # set by generate_random() or manually via set_goal()

    # Distance threshold in meters for reaching the goal
    GOAL_REACH_DIST = 3.0

    data = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0,
        0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0,
        0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0,
        0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0,
        0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0,
        0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0,
        0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0,
        0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0,
        0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0,
        0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0,
        0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ]

    @staticmethod
    def is_obstacle(x_m, y_m):
        """Check if a position (in meters) is an obstacle."""
        from math import floor
        x = int(floor(round(x_m) / Maze.TILESIZE))
        y = int(floor(round(y_m) / Maze.TILESIZE))
        return Maze.is_obstacle_raw(x, y)

    @staticmethod
    def is_obstacle_raw(x, y):
        """Check if a tile position is an obstacle."""
        if (x >= 0 and y >= 0 and
                x < Maze.width and y < Maze.height and
                Maze.data[x + (Maze.height - y - 1) * Maze.width]):
            return False
        return True

    @staticmethod
    def get_surrounding_obstacles_raw(x, y):
        obs = []
        for nx, ny in [(x, y + 1), (x, y - 1), (x - 1, y), (x + 1, y),
                       (x - 1, y + 1), (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1)]:
            obs.append(Maze.is_obstacle_raw(nx, ny))
        return obs

    @staticmethod
    def get_neighbour_cells(x, y):
        """Get neighbouring cell positions (in meters)."""
        from math import floor
        cx = int(floor(x / Maze.TILESIZE)) * Maze.TILESIZE
        cy = int(floor(y / Maze.TILESIZE)) * Maze.TILESIZE
        return (
            (cx - Maze.TILESIZE, cy),
            (cx, cy - Maze.TILESIZE),
            (cx, cy + Maze.TILESIZE),
            (cx + Maze.TILESIZE, cy),
        )

    @staticmethod
    def set_goal(tile_x, tile_y):
        """Set the goal position (in tile coordinates)."""
        Maze.goal = (tile_x, tile_y)

    @staticmethod
    def clear_goal():
        """Disable goal rendering/checking."""
        Maze.goal = None

    @staticmethod
    def set_goal_top_right_open():
        """Set goal to the top-right-most open tile. Returns (x, y) or None."""
        for y in range(Maze.height - 1, -1, -1):
            for x in range(Maze.width - 1, -1, -1):
                if not Maze.is_obstacle_raw(x, y):
                    Maze.set_goal(x, y)
                    return (x, y)
        Maze.goal = None
        return None

    @staticmethod
    def goal_position_meters():
        """Return goal position in meters (center of tile), or None."""
        if Maze.goal is None:
            return None
        gx, gy = Maze.goal
        return ((gx + 0.5) * Maze.TILESIZE, (gy + 0.5) * Maze.TILESIZE)

    @staticmethod
    def check_goal_reached(robot_x, robot_y):
        """Check if a robot (position in meters) has reached the goal."""
        goal_m = Maze.goal_position_meters()
        if goal_m is None:
            return False
        gx, gy = goal_m
        dist = ((robot_x - gx) ** 2 + (robot_y - gy) ** 2) ** 0.5
        return dist < Maze.GOAL_REACH_DIST

    @staticmethod
    def generate_random(seed=None):
        """Generate a random solvable maze using recursive backtracking.

        Replaces Maze.data with a new random maze. Sets start at bottom-left
        and goal at top-right. Returns the seed used.
        """
        if seed is None:
            seed = random.randint(0, 999999)
        rng = random.Random(seed)

        w, h = Maze.width, Maze.height

        # Start with all walls
        grid = [[0] * w for _ in range(h)]

        # Recursive backtracking on odd-indexed cells
        # We carve passages between cells that are 2 apart
        def carve(cx, cy):
            grid[cy][cx] = 1
            directions = [(0, 2), (0, -2), (2, 0), (-2, 0)]
            rng.shuffle(directions)
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if 1 <= nx < w - 1 and 1 <= ny < h - 1 and grid[ny][nx] == 0:
                    # Carve the wall between current and next
                    grid[cy + dy // 2][cx + dx // 2] = 1
                    carve(nx, ny)

        # Start carving from (1, 1)
        carve(1, 1)

        # Open up some extra passages to make it less of a perfect maze
        # (more fun for wall followers — multiple paths)
        extra_openings = (w * h) // 8
        for _ in range(extra_openings):
            x = rng.randrange(2, w - 2)
            y = rng.randrange(2, h - 2)
            if grid[y][x] == 0:
                # Only open if it connects two open cells
                neighbors_open = sum(1 for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]
                                     if 0 <= x + dx < w and 0 <= y + dy < h
                                     and grid[y + dy][x + dx] == 1)
                if neighbors_open >= 2:
                    grid[y][x] = 1

        # Ensure start and goal areas are open
        grid[h - 2][1] = 1  # bottom-left (tile 1,1 in y-up coords)
        grid[1][w - 2] = 1  # top-right

        # Convert to flat array (the grid is stored top-to-bottom,
        # which matches the data layout: row 0 = top of maze)
        Maze.data = []
        for row in grid:
            Maze.data.extend(row)

        # Set goal at the top-right-most open tile in the generated map.
        Maze.set_goal_top_right_open()

        return seed

    @staticmethod
    def get_edge_tile(x, y):
        up, down, left, right, upleft, upright, downleft, downright = \
            Maze.get_surrounding_obstacles_raw(x, y)

        if up and down and left and right:
            return None
        if up and down and left and not right:
            return TILES[RIGHT]
        if up and down and not left and right:
            return TILES[LEFT]
        if up and not down and left and right:
            return TILES[BOTTOM]
        if not up and down and left and right:
            return TILES[TOP]
        if up and not down and not left and right:
            return TILES[BOTTOM_LEFT_CVX]
        if up and not down and left and not right:
            return TILES[BOTTOM_RIGHT_CVX]
        if not up and down and not left and right:
            return TILES[TOP_LEFT_CVX]
        if not up and down and left and not right:
            return TILES[TOP_RIGHT_CVX]
        if not up and not down and left and not right:
            return TILES[BOTTOM_TOP_RIGHT]
        if not up and not down and not left and right:
            return TILES[BOTTOM_TOP_LEFT]
        if not up and not down and left and right:
            return TILES[BOTTOM_TOP]
        if up and down and not left and not right:
            return TILES[LEFT_RIGHT]
        if not up and down and not left and not right:
            return TILES[TOP_LEFT_RIGHT]
        if up and not down and not left and not right:
            return TILES[BOTTOM_LEFT_RIGHT]

        return None

    @staticmethod
    def overlay_transparent(background, overlay, x, y):
        background_width = background.shape[1]
        background_height = background.shape[0]

        if x >= background_width or y >= background_height:
            return background

        h, w = overlay.shape[0], overlay.shape[1]

        if x + w > background_width:
            w = background_width - x
            overlay = overlay[:, :w]

        if y + h > background_height:
            h = background_height - y
            overlay = overlay[:h]

        if overlay.shape[2] < 4:
            overlay = np.concatenate(
                [overlay, np.ones((overlay.shape[0], overlay.shape[1], 1),
                                  dtype=overlay.dtype) * 255],
                axis=2,
            )

        overlay_image = overlay[..., :3]
        mask = overlay[..., 3:] / 255.0

        background[y:y + h, x:x + w] = \
            (1.0 - mask) * background[y:y + h, x:x + w] + mask * overlay_image

        return background

    @staticmethod
    def render(robots):
        from math import pi

        # Always generate the same random floor pattern
        random.seed(1)

        img = np.zeros(
            (Maze.height * Maze.TILESIZE * Maze.M2PX,
             Maze.width * Maze.TILESIZE * Maze.M2PX, 3),
            np.uint8)
        img[:] = (140, 178, 250)

        for y in range(Maze.height):
            for x in range(Maze.width):
                px = x * Maze.TILESIZE * Maze.M2PX
                px2 = (x + 1) * Maze.TILESIZE * Maze.M2PX
                py = (Maze.height - y - 1) * Maze.TILESIZE * Maze.M2PX
                py2 = (Maze.height - y) * Maze.TILESIZE * Maze.M2PX

                if Maze.is_obstacle(x * Maze.TILESIZE, y * Maze.TILESIZE):
                    tile = Maze.get_edge_tile(x, y)
                    if tile is not None:
                        img[py:py2, px:px2] = tile
                    else:
                        cv2.rectangle(img, (px, py), (px2, py2), (0, 0, 0), -1)
                else:
                    img[py:py2, px:px2] = random.choice(TILES[FLOOR])

        # Draw goal marker
        if Maze.goal is not None:
            goal_m = Maze.goal_position_meters()
            gx_px = int(goal_m[0] * Maze.M2PX)
            gy_px = int((Maze.height * Maze.TILESIZE - goal_m[1]) * Maze.M2PX)
            radius = int(Maze.TILESIZE * Maze.M2PX * 0.35)
            cv2.circle(img, (gx_px, gy_px), radius, (0, 255, 0), -1)
            cv2.circle(img, (gx_px, gy_px), radius, (0, 180, 0), 2)
            cv2.putText(img, "GOAL",
                        (gx_px - radius, gy_px + radius + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)

        for name, robot in list(robots.items()):
            X = int(robot.x * Maze.M2PX)
            Y = int((Maze.height * Maze.TILESIZE - robot.y) * Maze.M2PX)

            for hit in robot.hitpoints:
                hx = int(hit[0] * Maze.M2PX)
                hy = int(hit[1] * Maze.M2PX)
                cv2.line(img, (X, Y), (hx + X, -hy + Y), (180, 180, 180), 1)
                cv2.circle(img, (hx + X, -hy + Y), 2, (220, 220, 220), -1)

            rotmat = cv2.getRotationMatrix2D(
                (ROBOT.shape[0] / 2, ROBOT.shape[1] / 2),
                robot.theta * 180 / pi, 1)
            rotated_robot = cv2.warpAffine(ROBOT, rotmat, ROBOT.shape[:2])
            w, h, _ = rotated_robot.shape

            Maze.overlay_transparent(img, rotated_robot, X - w // 2, Y - h // 2)

            cv2.putText(img, name,
                        (X, Y + int(Robot_RADIUS * Maze.M2PX) + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 200, 180), 2)

        return img

    @staticmethod
    def show(robots):
        img = Maze.render(robots)
        cv2.imshow('Maze', img)
        cv2.waitKey(1)
