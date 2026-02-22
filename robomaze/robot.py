import time
import numpy as np
from math import cos, sin, pi, floor, sqrt

import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.qos import qos_profile_sensor_data

from robomaze.maze import Maze


class Robot:

    RADIUS = 0.25 * Maze.TILESIZE
    MAXLIFE = 10

    def __init__(self, node, name="WallE", x=0., y=0., theta=0.):
        """Create a robot in the maze.

        Args:
            node: the rclpy Node instance (for creating publishers/subscribers)
            name: robot name (used for topic namespacing)
            x, y, theta: initial pose in meters / radians
        """
        self.node = node
        self.name = name
        self.base_frame = f"{self.name}_base_link"

        self.life = Robot.MAXLIFE

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

        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster(node)

        # Odometry publisher
        self.odom_pub = node.create_publisher(
            Odometry, f'{self.name}/odom', 10)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = f"{self.name}_odom"
        self.odom_msg.child_frame_id = self.base_frame
        self.odom_msg.pose.pose.position.z = 0.
        self.odom_msg.pose.pose.orientation.x = 0.
        self.odom_msg.pose.pose.orientation.y = 0.

        # LaserScan publisher (use sensor QoS for RViz2 compatibility)
        self.scan_pub = node.create_publisher(
            LaserScan, f'{self.name}/scan', qos_profile_sensor_data)
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = self.base_frame
        self.scan_msg.angle_min = self.angle_min
        self.scan_msg.angle_max = self.angle_max
        self.scan_msg.angle_increment = self.angle_increment
        self.scan_msg.time_increment = 0.
        self.scan_msg.range_min = self.range_min
        self.scan_msg.range_max = self.range_max

        # cmd_vel subscriber
        self.cmdvel_sub = node.create_subscription(
            Twist, f'{self.name}/cmd_vel', self.cmd_vel, 10)

        self.last = time.time()

    def setpose(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def cmd_vel(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def step(self, publish_odom=False, publish_tf=False, publish_laser=False):

        now = time.time()
        dt = now - self.last
        self.last = now

        ntheta = self.theta + dt * self.w
        nx = self.x + dt * cos(ntheta) * self.v
        ny = self.y + dt * sin(ntheta) * self.v

        head_x = nx + cos(ntheta) * Robot.RADIUS
        head_y = ny + sin(ntheta) * Robot.RADIUS

        # Move only if we don't hit an obstacle
        if not Maze.is_obstacle(head_x, head_y):
            self.theta = ntheta
            self.x = nx
            self.y = ny
            self.ranges, self.hitpoints = self.raycasting()

        # Quaternion from yaw (roll=0, pitch=0)
        qx = 0.0
        qy = 0.0
        qz = sin(self.theta / 2.0)
        qw = cos(self.theta / 2.0)

        stamp = self.node.get_clock().now().to_msg()

        if publish_odom:
            self.odom_msg.header.stamp = stamp
            self.odom_msg.pose.pose.position.x = self.x
            self.odom_msg.pose.pose.position.y = self.y
            self.odom_msg.pose.pose.orientation.z = qz
            self.odom_msg.pose.pose.orientation.w = qw
            self.odom_msg.twist.twist.linear.x = self.v
            self.odom_msg.twist.twist.angular.z = self.w
            self.odom_pub.publish(self.odom_msg)

        if publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = f"{self.name}_odom"
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.br.sendTransform(t)

        if publish_laser:
            self.scan_msg.header.stamp = stamp
            self.scan_msg.scan_time = dt
            self.scan_msg.ranges = self.ranges
            self.scan_pub.publish(self.scan_msg)

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
        return (Robot.ccw(A, C, D) != Robot.ccw(B, C, D) and
                Robot.ccw(A, B, C) != Robot.ccw(A, B, D))

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
            intersect = Robot.segment_intersect(R0, R1, *s)
            if intersect:
                intersection = Robot.line_intersection(s, (R0, R1))
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
