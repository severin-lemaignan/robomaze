from math import cos, sin, pi

import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.qos import qos_profile_sensor_data

from robomaze.robot import RobotCore


class RosRobot(RobotCore):
    """ROS2 transport layer on top of RobotCore.

    Adds publishers (odom, scan), subscriber (cmd_vel), and TF broadcaster.
    """

    def __init__(self, node, name="WallE", x=0., y=0., theta=0.):
        super().__init__(name, x, y, theta)
        self.node = node

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

    def cmd_vel(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def step(self, publish_odom=False, publish_tf=False, publish_laser=False):
        dt = super().step()

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
