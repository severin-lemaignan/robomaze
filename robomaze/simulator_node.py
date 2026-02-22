import os
import time
from math import pi

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from robomaze.maze import Maze, init_assets
from robomaze.robot import Robot


class MazeSimulatorNode(Node):

    def __init__(self):
        super().__init__('robomaze')

        # Parameters
        self.declare_parameter('random_maze', False)
        self.declare_parameter('seed', -1)
        self.declare_parameter('enable_goal', False)

        # Load tile and robot images
        pkg_share = get_package_share_directory('robomaze')
        res_root = os.path.join(pkg_share, 'res') + '/'
        init_assets(res_root)

        # Generate random maze if requested
        use_random = self.get_parameter('random_maze').value
        seed_param = self.get_parameter('seed').value
        enable_goal = self.get_parameter('enable_goal').value

        if use_random:
            seed = seed_param if seed_param >= 0 else None
            actual_seed = Maze.generate_random(seed)
            self.get_logger().info(
                f'Generated random maze with seed: {actual_seed}')
        else:
            self.get_logger().info('Using default hardcoded maze.')

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

        # Subscribe to robot creation requests
        self.create_sub = self.create_subscription(
            String, 'create_robot', self.on_new_robot, 10)

        # Publisher for goal reached announcements
        self.goal_pub = self.create_publisher(String, 'goal_reached', 10)
        self.winners_pub = self.create_publisher(String, 'winners', 10)

        # 10 Hz simulation loop
        self.timer = self.create_timer(0.1, self.timer_callback)

        cv2.namedWindow("Maze", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info('Robomaze simulator started!')
        self.get_logger().info(
            'Publish to /create_robot to spawn a robot, e.g.:')
        self.get_logger().info(
            "  ros2 topic pub --once /create_robot std_msgs/msg/String "
            "\"data: 'WallE'\"")

    def on_new_robot(self, msg):
        name = msg.data
        if name in self.robots:
            self.get_logger().error(f'Robot <{name}> already exists!')
            return

        # All robots spawn at bottom-left (tile 1,1) for a fair race
        spawn_x = 1.5 * Maze.TILESIZE
        spawn_y = 1.5 * Maze.TILESIZE
        spawn_theta = 0.0

        self.robots[name] = Robot(
            self, name, spawn_x, spawn_y, spawn_theta)

        self.get_logger().info(f'Created robot: {name}')

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

        Maze.show(self.robots)


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
