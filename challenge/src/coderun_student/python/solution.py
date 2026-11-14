#!/usr/bin/env python3
"""
CodeRun - your solution node.

TASK: the judge gives you a list of goal poses (x, y, yaw). Make the robot reach them
IN ORDER, as FAST as you can, without touching anything. A goal counts when the robot is
within 0.3 m of it AND its heading is within 20 deg of the goal yaw.

The code is written for you; you complete the lines marked TODO (search for "TODO"):
    EASY  TODO 1    plan_waypoints()    the points to drive through
          TODO 2-6  follow_waypoint()   the speeds that bring the robot there
    HARD  set USE_GRID_PATH = True below, then
          TODO 7    plan_waypoints()    the points along a path on the grid
          TODO 8    grid_path()         the search that finds that path
Do them in order: after TODO 1-6 EASY is solved, and the same driving code works for HARD. Run it after every TODO and watch the robot.
You don't need to know ROS: the robot's pose is in self.x, self.y, self.yaw and the goals in
self.goals. Everything at the bottom (marked "ROS") is done for you.

Coordinates: metres, x to the right and y up (as in RViz), angles in radians, 0 = facing +x,
positive = counter-clockwise (turning left). The robot starts at (0, 0) facing +x.

Run it with (no rebuild needed after an edit, just run it again):
    ros2 run coderun_student solution.py
"""

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from sensor_msgs.msg import LaserScan

# ---- settings you can change ----
USE_GRID_PATH = False  # False = EASY (corner points, TODO 1), True = HARD (grid path, TODO 7 + 8)
ARRIVED = 0.1          # m: closer than this to a waypoint = "at the point"
FACING = 0.1           # rad (about 6 deg): turned less than this away from the point = "facing it"

# ---- fixed ----
MAX_LINEAR = 1.0     # m/s, the simulator's limit: faster speeds are cut to this
MAX_ANGULAR = 0.78   # rad/s, same for turning
CELL = 0.5           # planning grid cell size (m)
CLEARANCE = 0.35     # a cell is free when its centre is at least this far from any wall (m)


def clamp(value, limit):
    """Limit value to [-limit, +limit]. Example: clamp(2.0, 0.5) == 0.5."""
    return max(-limit, min(limit, value))


def wrap(angle):
    """Wrap an angle to [-pi, pi], so that e.g. 350 deg becomes -10 deg (turn the short way)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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

    # ===================================================== STEP 1: plan the waypoints ==
    def plan_waypoints(self):
        """Called once at the start. Returns the points to drive through, in order:
        a list of (x, y, yaw). Goals keep their yaw (the robot must turn to it there),
        points in between get yaw = None (just drive through them)."""
        waypoints = []
        prev_x, prev_y = self.x, self.y          # the robot starts here

        for gx, gy, gyaw in self.goals:          # the goals, in order
            if not USE_GRID_PATH:
                # EASY: drive first along x, then along y. That never hits a wall in EASY.
                # TODO 1: the corner point between (prev_x, prev_y) and the goal (gx, gy).
                #         Which x and which y does it have?
                corner_x, corner_y = gx, gy      # <- fix this (now it goes straight to the goal)
                waypoints.append((corner_x, corner_y, None))
            else:
                # HARD: the straight line is always blocked, so follow a path on the grid.
                cells = self.grid_path(self.world_to_cell(prev_x, prev_y), self.world_to_cell(gx, gy))
                self.print_grid(cells)
                # TODO 7 (HARD): add the centre of every cell on the path as a waypoint (yaw None).
                #         self.cell_to_world(row, col) gives the centre (x, y) of a cell.
                for row, col in cells:
                    pass                         # <- replace with your code

            waypoints.append((gx, gy, gyaw))     # the goal itself
            prev_x, prev_y = gx, gy              # the next leg starts at this goal
        return waypoints

    # ================================================ STEP 2: drive to one waypoint ==
    def follow_waypoint(self, wx, wy, wyaw):
        """Called 10 times a second with the waypoint the robot is driving to.
        Returns (linear, angular, reached):
            linear    forward speed, m/s           (the simulator caps it at 1.0)
            angular   turning speed, rad/s, + = left (capped at 0.78)
            reached   True = done with this waypoint, the next one is used from now on"""
        distance = self.distance_to(wx, wy)      # metres to the point
        angle = self.angle_to(wx, wy)            # how much to turn to face it (rad, + = left)

        if distance > ARRIVED:                   # A. not there yet
            if abs(angle) > FACING:              # A1. not facing the point: turn on the spot
                # TODO 2: stop (linear = 0) and turn towards the point.
                #         Hint: angular = a number * angle (the further off, the faster you turn)
                linear = 0.2                     # <- fix this
                angular = 0.0                    # <- fix this
            else:                                # A2. facing it: drive
                # TODO 3: drive forward. Faster when far, slower when close (but at least 0.1 m/s).
                linear = 0.2                     # <- try something with distance
                # TODO 4: small steering correction, so you keep facing the point.
                angular = 0.0                    # <- try something with angle
            return linear, angular, False

        # B. at the point
        if wyaw is None:                         # a point in between: nothing more to do
            return 0.0, 0.0, True

        error = self.heading_error(wyaw)         # how much to turn to the goal yaw (rad, + = left)
        # TODO 5: turn on the spot towards the goal yaw (like TODO 2, with error).
        angular = 0.0                            # <- fix this
        # TODO 6: reached when the heading is close enough. The judge allows 20 deg = 0.35 rad,
        #         but aim for much less (odometry is not perfect).
        reached = False                          # <- fix this
        return 0.0, angular, reached

    # ================================================== HARD: search the planning grid ==
    def grid_path(self, start, goal):
        """Find a path on self.grid from cell start to cell goal (cells are (row, col)).
        Return the cells of the path, start first, goal last: [start, ..., goal]."""
        # TODO 8 (HARD): write a breadth-first search (BFS). The idea:
        #   - keep a queue of cells to look at, starting with [start] (deque is imported for you)
        #   - keep a dict came_from: cell -> the cell you reached it from (start: None)
        #   - take the oldest cell from the queue; stop when it is the goal
        #   - for every cell in self.free_neighbours(cell) that is not in came_from yet:
        #     remember where you came from and add it to the queue
        #   - at the end, walk back from goal to start with came_from, and reverse that list
        # BFS finds the path with the fewest cells. Check your path with self.print_grid(path).
        return [start, goal]                     # <- placeholder: straight to the goal (hits walls)

    # =================================================================== helpers ==
    def distance_to(self, x, y):
        """Distance in metres from the robot to the point (x, y)."""
        return math.hypot(x - self.x, y - self.y)

    def angle_to(self, x, y):
        """How much the robot has to turn to face the point (x, y): rad, + = left, - = right."""
        return wrap(math.atan2(y - self.y, x - self.x) - self.yaw)

    def heading_error(self, yaw):
        """How much the robot has to turn to face the direction yaw: rad, + = left, - = right."""
        return wrap(yaw - self.yaw)

    def world_to_cell(self, x, y):
        """(x, y) in metres -> (row, col) of the planning grid cell that contains it."""
        o = self.map.info.origin.position
        return int((y - o.y) // CELL), int((x - o.x) // CELL)

    def cell_to_world(self, row, col):
        """(row, col) of the planning grid -> (x, y) in metres of the cell centre."""
        o = self.map.info.origin.position
        return o.x + (col + 0.5) * CELL, o.y + (row + 0.5) * CELL

    def free_neighbours(self, cell):
        """The free cells next to cell = (row, col): up, down, left and right (no diagonals)."""
        r, c = cell
        out = []
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= nr < len(self.grid) and 0 <= nc < len(self.grid[0]) and self.grid[nr][nc]:
                out.append((nr, nc))
        return out

    def print_grid(self, path=()):
        """Print the grid, top row = highest y (as in RViz): '#' blocked, '.' free,
        'R' robot, '1'.. goals, '*' cells of `path` (a list of (row, col))."""
        marks = {cell: '*' for cell in path}
        marks[self.world_to_cell(self.x, self.y)] = 'R'
        for i, (gx, gy, _) in enumerate(self.goals or [], start=1):
            marks[self.world_to_cell(gx, gy)] = str(i % 10)
        for r in reversed(range(len(self.grid))):
            print(''.join(marks.get((r, c), '.' if free else '#') for c, free in enumerate(self.grid[r])))

    # ============================================ ROS: you don't need to change this ==
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

        self.goals = None          # list of (x, y, yaw), in order
        self.map = None            # nav_msgs/OccupancyGrid (0.04 m cells, 100 = wall)
        self.grid = None           # planning grid, see build_grid()
        self.scan = None           # sensor_msgs/LaserScan (not needed, but there if you want it)
        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self.waypoints = None      # from plan_waypoints()
        self.index = 0             # the waypoint the robot is driving to now

    def goals_callback(self, msg):
        if self.goals is None:
            self.get_logger().info(f'received {len(msg.poses)} goals')
        self.goals = [(p.position.x, p.position.y, yaw_from_quaternion(p.orientation)) for p in msg.poses]

    def map_callback(self, msg):
        self.map = msg
        self.grid = build_grid(msg)

    def scan_callback(self, msg):
        # msg.ranges[i] is the distance at angle msg.angle_min + i * msg.angle_increment
        # (0 rad = straight ahead, positive = left)
        self.scan = msg

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def control_loop(self):
        """Runs 10 times a second: calls your STEP 1 once, then your STEP 2 for the current waypoint."""
        if self.goals is None or self.map is None or not self.have_odom:
            self.publish(0.0, 0.0)             # wait for the goals, the map and the robot pose
            return

        if self.waypoints is None:
            self.waypoints = self.plan_waypoints()
            self.print_grid()
            self.get_logger().info(f'{len(self.waypoints)} waypoints: '
                                   + ', '.join(f'({x:.1f}, {y:.1f})' for x, y, _ in self.waypoints))

        if self.index >= len(self.waypoints):
            self.publish(0.0, 0.0)             # all done: stop
            return

        wx, wy, wyaw = self.waypoints[self.index]
        linear, angular, reached = self.follow_waypoint(wx, wy, wyaw)
        if reached:
            self.index += 1
            self.get_logger().info(f'waypoint {self.index}/{len(self.waypoints)} reached')
            linear, angular = 0.0, 0.0
        self.publish(linear, angular)

    def publish(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(clamp(linear, MAX_LINEAR))
        cmd.angular.z = float(clamp(angular, MAX_ANGULAR))
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
