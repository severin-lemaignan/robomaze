import os
import random
import time
from math import pi

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from robomaze.maze import Maze, init_assets
from robomaze.ros_robot import RosRobot


class MazeSimulatorNode(Node):

    def __init__(self):
        super().__init__('robomaze')

        # Parameters
        self.declare_parameter('random_maze', False)
        self.declare_parameter('seed', -1)
        self.declare_parameter('enable_goal', False)
        self.declare_parameter('map_width', Maze.width)
        self.declare_parameter('map_height', Maze.height)

        # Load tile and robot images
        pkg_share = get_package_share_directory('robomaze')
        res_root = os.path.join(pkg_share, 'res') + '/'
        init_assets(res_root)

        # Generate random maze if requested
        use_random = self.get_parameter('random_maze').value
        seed_param = self.get_parameter('seed').value
        enable_goal = self.get_parameter('enable_goal').value
        map_width = int(self.get_parameter('map_width').value)
        map_height = int(self.get_parameter('map_height').value)

        if use_random:
            if map_width < 5 or map_height < 5:
                self.get_logger().warn(
                    'map_width/map_height must be >= 5. '
                    'Falling back to 20x20.'
                )
                map_width, map_height = 20, 20
            Maze.width = map_width
            Maze.height = map_height
            seed = seed_param if seed_param >= 0 else None
            actual_seed = Maze.generate_random(seed)
            self.get_logger().info(
                f'Generated random maze {Maze.width}x{Maze.height} '
                f'with seed: {actual_seed}')
        else:
            self.get_logger().info('Using default hardcoded maze.')
            if map_width != 20 or map_height != 20:
                self.get_logger().warn(
                    'map_width/map_height are ignored unless random_maze:=true '
                    '(fixed map is 20x20).'
                )

        if enable_goal:
            goal_tile = Maze.set_goal_top_right_open()
            self.get_logger().info(f'Goal enabled at tile: {goal_tile}')
        else:
            Maze.clear_goal()
            self.get_logger().info('Goal disabled (enable with -p enable_goal:=true)')

        self.robots = {}
        self.spawn_idx = 0
        self.finished_robots = set()
        self.winners = []

        # Subscribe to robot creation/deletion requests
        self.create_sub = self.create_subscription(
            String, 'create_robot', self.on_new_robot, 10)
        self.delete_sub = self.create_subscription(
            String, 'delete_robot', self.on_delete_robot, 10)

        # Publisher for goal reached announcements
        self.goal_pub = self.create_publisher(String, 'goal_reached', 10)
        self.winners_pub = self.create_publisher(String, 'winners', 10)

        # 10 Hz simulation loop
        self.timer = self.create_timer(0.1, self.timer_callback)

        cv2.namedWindow("Maze", cv2.WINDOW_GUI_NORMAL)
        self._fullscreen = False
        self.get_logger().info('Robomaze simulator started!')
        self.get_logger().info(
            'Publish to /create_robot to spawn a robot, e.g.:')
        self.get_logger().info(
            "  ros2 topic pub --once /create_robot std_msgs/msg/String "
            "\"data: 'WallE'\"")

    def _find_spawn_position(self, min_dist_goal=3, min_dist_robots=2):
        """Find a random valid spawn position (in tile coords), far enough
        from the goal and existing robots.

        Distances are in tiles.
        """
        open_tiles = []
        for y in range(Maze.height):
            for x in range(Maze.width):
                if not Maze.is_obstacle_raw(x, y):
                    open_tiles.append((x, y))

        random.shuffle(open_tiles)

        goal_m = Maze.goal_position_meters()
        for tx, ty in open_tiles:
            cx = (tx + 0.5) * Maze.TILESIZE
            cy = (ty + 0.5) * Maze.TILESIZE

            # Check distance to goal
            if goal_m is not None:
                dist = ((cx - goal_m[0]) ** 2 + (cy - goal_m[1]) ** 2) ** 0.5
                if dist < min_dist_goal * Maze.TILESIZE:
                    continue

            # Check distance to existing robots
            too_close = False
            for robot in self.robots.values():
                dist = ((cx - robot.x) ** 2 + (cy - robot.y) ** 2) ** 0.5
                if dist < min_dist_robots * Maze.TILESIZE:
                    too_close = True
                    break
            if too_close:
                continue

            return cx, cy

        # Fallback: if no tile satisfies constraints, pick any open tile
        tx, ty = random.choice(open_tiles)
        return (tx + 0.5) * Maze.TILESIZE, (ty + 0.5) * Maze.TILESIZE

    def on_new_robot(self, msg):
        name = msg.data
        if name in self.robots:
            self.get_logger().error(f'Robot <{name}> already exists!')
            return

        spawn_x, spawn_y = self._find_spawn_position()
        spawn_theta = random.uniform(0, 2 * pi)

        self.robots[name] = RosRobot(
            self, name, spawn_x, spawn_y, spawn_theta)

        self.get_logger().info(
            f'Created robot: {name} at tile '
            f'({int(spawn_x / Maze.TILESIZE)}, {int(spawn_y / Maze.TILESIZE)})')

    def on_delete_robot(self, msg):
        name = msg.data
        if name not in self.robots:
            self.get_logger().error(f'Robot <{name}> does not exist!')
            return

        robot = self.robots.pop(name)
        self.destroy_publisher(robot.odom_pub)
        self.destroy_publisher(robot.scan_pub)
        self.destroy_subscription(robot.cmdvel_sub)
        self.finished_robots.discard(name)
        self.get_logger().info(f'Deleted robot: {name}')

    def timer_callback(self):
        to_remove = []
        for name, robot in list(self.robots.items()):
            robot.step(publish_odom=True, publish_tf=True, publish_laser=True)

            # Check if robot reached the goal
            if name not in self.finished_robots and \
                    Maze.check_goal_reached(robot.x, robot.y):
                self.finished_robots.add(name)
                elapsed = time.time() - robot.created
                self.winners.append((name, elapsed))
                place = len(self.winners)
                msg = String()
                msg.data = name
                self.goal_pub.publish(msg)
                self.get_logger().info(
                    f'*** {name} reached the goal! '
                    f'Place: #{place}, Time: {elapsed:.1f}s ***')
                to_remove.append(name)

        for name in to_remove:
            self.robots.pop(name, None)
            self.get_logger().info(f'Removed finished robot: {name}')

        if to_remove:
            ranking = ', '.join(
                f'#{idx + 1} {winner} ({t:.1f}s)'
                for idx, (winner, t) in enumerate(self.winners)
            )
            self.get_logger().info(f'Winners so far: {ranking}')
            ranking_msg = String()
            ranking_msg.data = ranking
            self.winners_pub.publish(ranking_msg)

        img = Maze.render(self.robots)
        cv2.imshow('Maze', img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('f'):
            self._fullscreen = not self._fullscreen
            if self._fullscreen:
                cv2.setWindowProperty(
                    "Maze", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            else:
                cv2.setWindowProperty(
                    "Maze", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)


def main(args=None):
    rclpy.init(args=args)
    node = MazeSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
