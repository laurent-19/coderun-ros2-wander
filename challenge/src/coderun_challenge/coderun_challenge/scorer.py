"""
CodeRun automatic scorer.

Listens to the simulator ground truth (/ground_truth), detects collisions against the
very same bitmap Stage uses, tracks the goal poses reached and writes a JSON result.

A goal counts when, in order, the robot is within `goal_radius` of it AND its heading is
within `yaw_tolerance_deg` of the goal yaw (both read from config/<task>_goals.yaml).

Point system (from the CodeRun guidelines):

            max   baseline   requirements   time bonus
    EASY    110      10           90        0.5 / min left in the slot, max 10
    HARD    220      20          180        1.0 / min left in the slot, max 20

Requirements = max_req * goals_reached / goals_total - penalty * collisions (min 0).
Time bonus = rate * minutes left in the CodeRun slot when the team handed in its solution
(`slot_minutes_left`, set by the graders), capped, only awarded when the requirement points
are >= 50 % of their maximum. The robot run time (from the first motion) is a tie-breaker.
"""

import json
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy import ndimage

RES = 0.02            # bitmap resolution (m / px), must match the world files
COLLIDE_DIST = 0.28   # pose-to-wall distance below which the robot is touching
CLEAR_DIST = 0.34     # hysteresis: distance above which a new collision may count
MOVE_DIST = 0.02      # the run clock starts once the robot moved this far (m) ...
MOVE_YAW = 0.03       # ... or turned this much (rad)

MODES = {
    #        base  req_max  time_rate  time_max  penalty
    'easy': (10.0, 90.0, 0.5, 10.0, 5.0),
    'hard': (20.0, 180.0, 1.0, 20.0, 10.0),
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class Scorer(Node):
    def __init__(self):
        super().__init__('coderun_scorer')
        share = get_package_share_directory('coderun_challenge')
        self.declare_parameter('mode', 'easy')
        self.declare_parameter('team', 'unnamed')
        self.declare_parameter('time_limit', 1200.0)      # seconds (20 min) from the first motion
        self.declare_parameter('slot_minutes_left', 0.0)  # minutes left in the slot at hand-in
        self.declare_parameter('ground_truth_topic', 'ground_truth')
        self.declare_parameter('goals_file', '')
        self.declare_parameter('goals', Parameter.Type.DOUBLE_ARRAY)   # [x, y, yaw_rad, ...] of this run
        self.declare_parameter('seed', '')
        self.declare_parameter('bitmap', '')
        self.declare_parameter('result_file', '/tmp/coderun_result.json')

        self.mode = self.get_parameter('mode').value
        if self.mode not in MODES:
            raise ValueError(f'mode must be one of {list(MODES)}')
        self.base, self.req_max, self.time_rate, self.time_max, self.penalty = MODES[self.mode]
        self.team = self.get_parameter('team').value
        self.time_limit = float(self.get_parameter('time_limit').value)
        self.slot_left = float(self.get_parameter('slot_minutes_left').value)
        self.result_file = self.get_parameter('result_file').value

        goals_file = self.get_parameter('goals_file').value or f'{share}/config/{self.mode}_goals.yaml'
        cfg = yaml.safe_load(Path(goals_file).read_text())
        self.size = float(cfg['world_size'])
        # map centre in the start frame (the frame of the goals and of /ground_truth)
        self.ox = float(cfg['map_origin']['x'])
        self.oy = float(cfg['map_origin']['y'])
        self.radius = float(cfg['goal_radius'])
        self.yaw_tol = math.radians(cfg['yaw_tolerance_deg'])
        # goals of THIS run (random ones included) come from the launch file; without them only
        # the static goals of the YAML are used
        flat = list(self.get_parameter('goals').value or [])
        if flat:
            self.goals = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        else:
            self.goals = [(g['x'], g['y'], math.radians(g['yaw_deg'])) for g in cfg['goals']]
        self.seed = self.get_parameter('seed').value

        bitmap = self.get_parameter('bitmap').value or f'{share}/bitmaps/{self.mode}.png'
        occ = np.array(Image.open(bitmap).convert('L')) < 128
        self.dist = ndimage.distance_transform_edt(~occ) * RES
        self.get_logger().info(
            f'[{self.team}] mode={self.mode} goals={len(self.goals)} bitmap={Path(bitmap).name} '
            f'time_limit={self.time_limit:.0f}s')

        self.t_launch = None    # first ground truth message
        self.start_pose = None
        self.t0 = None          # first motion: the run clock starts here
        self.collisions = 0
        self.in_collision = False
        self.next_goal = 0
        self.goal_times = []
        self.finished = False
        self.last_log = -10.0
        self.gt_topic = self.get_parameter('ground_truth_topic').value
        self.create_subscription(Odometry, self.gt_topic, self.on_gt, 10)

    # ----------------------------------------------------------- helpers ---
    def _wall_dist(self, x, y):
        c = int((x - self.ox + self.size / 2) / RES)
        r = int((self.size / 2 - (y - self.oy)) / RES)
        if not (0 <= r < self.dist.shape[0] and 0 <= c < self.dist.shape[1]):
            return 0.0
        return float(self.dist[r, c])

    def progress(self):
        return self.next_goal / max(len(self.goals), 1)

    def score(self):
        req = max(0.0, self.req_max * self.progress() - self.penalty * self.collisions)
        time_pts = 0.0
        if req >= 0.5 * self.req_max:
            time_pts = min(self.time_max, self.time_rate * max(0.0, self.slot_left))
        return {
            'baseline': self.base,
            'requirements': round(req, 2),
            'time_bonus': round(time_pts, 2),
            'total': round(self.base + req + time_pts, 2),
            'max_total': self.base + self.req_max + self.time_max,
        }

    # ---------------------------------------------------------- callback ---
    def on_gt(self, msg):
        if self.finished:
            return
        # use wall clock time (Stage's sim time persists between runs)
        now = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        if self.t_launch is None:
            self.t_launch, self.start_pose = now, (x, y, yaw)
        if self.t0 is None:
            sx, sy, syaw = self.start_pose
            if math.hypot(x - sx, y - sy) > MOVE_DIST or angle_diff(yaw, syaw) > MOVE_YAW:
                self.t0 = now
                self.get_logger().info('robot moving: run clock started')
            elif now - self.t_launch >= self.time_limit:
                self.finish(0.0, False)         # never moved
                return
        elapsed = now - self.t0 if self.t0 is not None else 0.0

        # collisions (rising edge with hysteresis)
        d = self._wall_dist(x, y)
        if d < COLLIDE_DIST and not self.in_collision:
            self.in_collision = True
            self.collisions += 1
            self.get_logger().warn(f'collision #{self.collisions} at ({x:.2f}, {y:.2f}) t={elapsed:.0f}s')
        elif d > CLEAR_DIST:
            self.in_collision = False

        # goals, strictly in order
        if self.next_goal < len(self.goals):
            gx, gy, gyaw = self.goals[self.next_goal]
            if math.hypot(x - gx, y - gy) < self.radius and angle_diff(yaw, gyaw) < self.yaw_tol:
                self.next_goal += 1
                self.goal_times.append(round(elapsed, 1))
                self.get_logger().info(f'goal {self.next_goal}/{len(self.goals)} reached at t={elapsed:.1f}s')
        done = self.next_goal >= len(self.goals)

        if elapsed - self.last_log >= 5.0:
            self.last_log = elapsed
            s = self.score()
            self.get_logger().info(
                f't={elapsed:5.0f}s goals={self.next_goal}/{len(self.goals)} '
                f'collisions={self.collisions} score~{s["total"]:.1f}')

        if done or (self.t0 is not None and elapsed >= self.time_limit):
            self.finish(elapsed, done)

    def finish(self, elapsed, done):
        self.finished = True
        # more than Stage publishing the judge's pose = someone else is faking it
        gt_publishers = self.count_publishers(self.gt_topic)
        s = self.score()
        result = {
            'team': self.team,
            'mode': self.mode,
            'seed': self.seed,
            'goals': [[round(x, 2), round(y, 2), round(math.degrees(a))] for x, y, a in self.goals],
            'completed': bool(done),
            'elapsed_s': round(elapsed, 1),           # from the first motion (tie-breaker)
            'slot_minutes_left': self.slot_left,
            'collisions': self.collisions,
            'goals_reached': self.next_goal,
            'goals_total': len(self.goals),
            'goal_times_s': self.goal_times,
            'ground_truth_publishers': gt_publishers,  # must be 1 (Stage)
            'score': s,
        }
        Path(self.result_file).write_text(json.dumps(result, indent=2))
        self.get_logger().info(
            f'FINISHED team={self.team} mode={self.mode} completed={done} '
            f'score={s["total"]}/{s["max_total"]} -> {self.result_file}')
        if gt_publishers != 1:
            self.get_logger().error(f'{gt_publishers} publishers on {self.gt_topic}: run is NOT valid')


def main(args=None):
    rclpy.init(args=args)
    node = Scorer()
    try:
        # spin until the run is over (shutting down from inside a callback hangs rclpy)
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
