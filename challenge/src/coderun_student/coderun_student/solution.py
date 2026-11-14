"""
CodeRun - your solution node (starting point, based on Lab 2 "wander").

TASK: the judge gives you a list of goal poses (x, y, yaw). Make the robot reach them
IN ORDER, as FAST as you can, without touching anything. A goal counts when the robot is
within 0.3 m of it AND its heading is within 20 deg of the goal yaw.

The goals are RANDOM at every run (in EASY only the last one, in HARD all of them), so you
must read them at runtime - hardcoded positions will not work.

You have:
    /goals      geometry_msgs/PoseArray  the goal poses, in order (latched / transient local)
    /map        nav_msgs/OccupancyGrid   the arena, 0.04 m cells, 100 = wall (latched)
    /odom       nav_msgs/Odometry        robot pose estimate (starts at (0, 0, 0))
    /base_scan  sensor_msgs/LaserScan    270 deg field of view, 1 deg steps, 5 m range
and you publish:
    /cmd_vel    geometry_msgs/Twist      linear.x <= 1.0 m/s, angular.z <= 0.78 rad/s

/goals, /map and /odom are all in the same frame (the robot starts at its origin).
/ground_truth is for the judge only - do not rely on it.

Plan waypoints that lead from goal to goal, then drive through them:
    EASY  driving first along x, then along y from goal to goal always works.
    HARD  a straight line to the next goal is always blocked. This template turns /map into a
          small planning grid for you: self.grid[row][col] is True where the robot can drive
          (0.5 m cells, walls already inflated), see build_grid(). Search it from the robot's
          cell to the goal's cell (e.g. breadth-first search, 4 neighbours), then drive through
          the centres of the cells on the path: the lines between neighbouring free cell
          centres keep clear of the walls.
          world_to_cell() / cell_to_world() convert between metres and grid cells, and
          print_grid() shows the grid in the terminal.

Run it with (the workspace is built with --symlink-install, so edits apply without a rebuild):
    ros2 run coderun_student solution
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from sensor_msgs.msg import LaserScan

CELL = 0.5           # planning grid cell size (m)
CLEARANCE = 0.35     # a cell is free when its centre is at least this far from any wall (m)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def build_grid(map_msg):
    """Planning grid from /map: grid[row][col] is True when the robot can stand at the centre
    of that CELL x CELL square. Row 0 is the lowest y, column 0 the lowest x (like /map)."""
    info = map_msg.info
    walls = np.array(map_msg.data, dtype=np.int16).reshape(info.height, info.width) > 50
    to_wall = ndimage.distance_transform_edt(~walls) * info.resolution    # metres to the nearest wall
    rows = int(info.height * info.resolution / CELL)
    cols = int(info.width * info.resolution / CELL)
    grid = []
    for r in range(rows):
        row = []
        for c in range(cols):
            mr = int((r + 0.5) * CELL / info.resolution)     # /map cell under the centre
            mc = int((c + 0.5) * CELL / info.resolution)
            row.append(bool(to_wall[mr, mc] >= CLEARANCE))
        grid.append(row)
    return grid


class SolutionNode(Node):
    def __init__(self):
        super().__init__('solution_node')
        # /goals and /map are published once and latched: ask for transient-local durability
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(PoseArray, 'goals', self.goals_callback, latched)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, latched)
        self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(0.1, self.control_loop)          # 10 Hz

        self.goals = None          # list of (x, y, yaw in radians), in order
        self.map = None            # nav_msgs/OccupancyGrid
        self.grid = None           # planning grid, see build_grid()
        self.scan = None
        self.x = self.y = self.yaw = 0.0
        self.waypoints = None

    def goals_callback(self, msg):
        self.goals = [(p.position.x, p.position.y, yaw_from_quaternion(p.orientation)) for p in msg.poses]
        self.get_logger().info(f'received {len(self.goals)} goals')

    def map_callback(self, msg):
        # msg.data[row * msg.info.width + col] is 100 for a wall, 0 for free space.
        # Cell (row, col) covers x in msg.info.origin.position.x + col * msg.info.resolution (+ one cell)
        # and y likewise with row. Row 0 is the lowest y.
        self.map = msg
        self.grid = build_grid(msg)

    def scan_callback(self, msg):
        # msg.ranges[i] is the distance at angle msg.angle_min + i * msg.angle_increment
        # (0 rad = straight ahead, positive = counter-clockwise / left)
        self.scan = msg

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    # ------------------------------------------------------------- grid helpers --
    def world_to_cell(self, x, y):
        """(x, y) in metres -> (row, col) of the planning grid cell that contains it."""
        o = self.map.info.origin.position
        return int((y - o.y) // CELL), int((x - o.x) // CELL)

    def cell_to_world(self, row, col):
        """(row, col) of the planning grid -> (x, y) in metres of the cell centre."""
        o = self.map.info.origin.position
        return o.x + (col + 0.5) * CELL, o.y + (row + 0.5) * CELL

    def print_grid(self, path=()):
        """Print the grid, top row = highest y (as in RViz): '#' blocked, '.' free,
        'R' robot, '1'.. goals, '*' cells of `path` (a list of (row, col))."""
        marks = {cell: '*' for cell in path}
        marks[self.world_to_cell(self.x, self.y)] = 'R'
        for i, (gx, gy, _) in enumerate(self.goals or [], start=1):
            marks[self.world_to_cell(gx, gy)] = str(i % 10)
        for r in reversed(range(len(self.grid))):
            print(''.join(marks.get((r, c), '.' if free else '#') for c, free in enumerate(self.grid[r])))

    # ------------------------------------------------------------------ control --
    def control_loop(self):
        cmd = Twist()
        if self.goals is None or self.map is None:
            self.cmd_pub.publish(cmd)          # wait for the goals and the map
            return

        if self.waypoints is None:
            self.print_grid()
            # TODO: turn self.goals into a list of waypoints: intermediate points that lead
            #       around the walls from one goal to the next (HARD: search self.grid)
            self.waypoints = list(self.goals)

        # TODO: compute the velocity that follows self.waypoints
        #       (turn towards the next waypoint, drive, turn to the goal yaw, ...)
        cmd.linear.x = 0.2         # placeholder: just drive forward
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SolutionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
