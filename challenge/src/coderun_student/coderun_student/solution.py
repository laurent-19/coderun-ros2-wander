"""
CodeRun - your solution node.

Run with:
    python3 ~/ros_ws/src/coderun_student/coderun_student/solution.py
"""

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class SolutionNode(Node):
    def __init__(self):
        super().__init__('solution_node')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(PoseArray, 'goals', self.goals_callback, latched)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, latched)
        self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(0.1, self.control_loop)

        self.goals = None
        self.map = None
        self.scan = None
        self.x = self.y = self.yaw = 0.0

    def goals_callback(self, msg):
        self.goals = msg.poses
        self.get_logger().info(f'{len(self.goals)} goals')

    def map_callback(self, msg):
        self.map = msg

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def control_loop(self):
        cmd = Twist()
        cmd.linear.x = 0.2
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SolutionNode())


if __name__ == '__main__':
    main()
