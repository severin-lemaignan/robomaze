import time
import numpy as np
from math import cos, sin, pi, floor, sqrt

from robomaze.maze import Maze


class RobotCore:
    """Pure game logic: kinematics, raycasting, collision detection.

    No ROS2 dependencies — safe to import from Flask or any other context.
    """

    RADIUS = 0.25 * Maze.TILESIZE
    MAXLIFE = 10

    def __init__(self, name="WallE", x=0., y=0., theta=0.):
        self.name = name
        self.base_frame = f"{self.name}_base_link"

        self.life = RobotCore.MAXLIFE

        self.lastinteraction = 0
        self.created = time.time()
        self.age = 0

        self.x = x
        self.y = y
        self.theta = theta

        self.v = 0.
        self.w = 0.

        # Laser scan parameters
        self.angle_min = -45. * pi / 180
        self.angle_max = 46. * pi / 180
        self.angle_increment = 2. * pi / 180
        self.range_min = 0.  # m
        self.range_max = 60.  # m

        self.ranges = []
        self.hitpoints = []

        self.last = time.time()

    def setpose(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def step(self):
        """Advance kinematics, check collision, update raycasting."""
        now = time.time()
        dt = now - self.last
        self.last = now

        ntheta = self.theta + dt * self.w
        nx = self.x + dt * cos(ntheta) * self.v
        ny = self.y + dt * sin(ntheta) * self.v

        head_x = nx + cos(ntheta) * RobotCore.RADIUS
        head_y = ny + sin(ntheta) * RobotCore.RADIUS

        # Move only if we don't hit an obstacle
        if not Maze.is_obstacle(head_x, head_y):
            self.theta = ntheta
            self.x = nx
            self.y = ny
            self.ranges, self.hitpoints = self.raycasting()

        return dt

    # --- Raycasting (pure math, no ROS dependency) ---

    @staticmethod
    def line_intersection(line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return None

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y

    @staticmethod
    def ccw(A, B, C):
        Ax, Ay = A
        Bx, By = B
        Cx, Cy = C
        return (Cy - Ay) * (Bx - Ax) > (By - Ay) * (Cx - Ax)

    @staticmethod
    def segment_intersect(A, B, C, D):
        """Return true if line segments AB and CD intersect."""
        return (RobotCore.ccw(A, C, D) != RobotCore.ccw(B, C, D) and
                RobotCore.ccw(A, B, C) != RobotCore.ccw(A, B, D))

    def get_ray_point(self, d, angular_offset=0.):
        return (self.x + cos(self.theta + angular_offset) * d,
                self.y + sin(self.theta + angular_offset) * d)

    def dist_to_robot(self, point):
        px, py = point
        return (self.x - px) ** 2 + (self.y - py) ** 2

    def cell_intersection(self, maze_x, maze_y, alpha=0.):
        A = maze_x, maze_y
        B = (maze_x + Maze.TILESIZE), maze_y
        C = (maze_x + Maze.TILESIZE), (maze_y + Maze.TILESIZE)
        D = maze_x, (maze_y + Maze.TILESIZE)

        R0 = self.x, self.y
        R1 = self.get_ray_point(self.range_max, angular_offset=alpha)

        intersections = {}

        for s, direction in [((A, B), "up"), ((B, C), "right"),
                             ((C, D), "down"), ((D, A), "left")]:
            intersect = RobotCore.segment_intersect(R0, R1, *s)
            if intersect:
                intersection = RobotCore.line_intersection(s, (R0, R1))
                intersections[self.dist_to_robot(intersection)] = \
                    (intersection, direction)

        # The ray does not cross this cell
        if not intersections:
            return None

        max_dist = max(intersections.keys())
        min_dist = min(intersections.keys())

        # Our ray did not traverse the cell: out of range
        if len(intersections) != 2:
            return self.range_max * self.range_max + 1, None

        # The ray crosses the cell, but the cell is empty — recurse
        if not Maze.is_obstacle(maze_x, maze_y):
            intersection, direction = intersections[max_dist]
            next_maze_x, next_maze_y = maze_x, maze_y
            if direction == "up":
                next_maze_y -= Maze.TILESIZE
            if direction == "down":
                next_maze_y += Maze.TILESIZE
            if direction == "right":
                next_maze_x += Maze.TILESIZE
            if direction == "left":
                next_maze_x -= Maze.TILESIZE

            return self.cell_intersection(next_maze_x, next_maze_y, alpha)

        # The ray hits an obstacle — return distance + hit point relative to robot
        x, y = intersections[min_dist][0]
        return min_dist, (x - self.x, y - self.y)

    def raycasting(self):
        rays = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        ranges = []
        hitpoints = []
        for alpha in rays:
            for maze_x, maze_y in Maze.get_neighbour_cells(self.x, self.y):
                hit = self.cell_intersection(maze_x, maze_y, alpha)
                if hit is not None:
                    dist, point = hit
                    ranges.append(sqrt(dist))
                    if point is not None:
                        hitpoints.append(point)
                    break

        return ranges, hitpoints
