// CodeRun - your solution node in C++ (the same as python/solution.py: pick either one).
//
// TASK: the judge gives you a list of goal poses (x, y, yaw). Make the robot reach them
// IN ORDER, as FAST as you can, without touching anything. A goal counts when the robot is
// within 0.3 m of it AND its heading is within 20 deg of the goal yaw.
//
// The code is written for you; you complete the lines marked TODO (search for "TODO"):
//     EASY  TODO 1    plan_waypoints()    the points to drive through
//           TODO 2-6  follow_waypoint()   the speeds that bring the robot there
//     HARD  set USE_GRID_PATH = true below, then
//           TODO 7    plan_waypoints()    the points along a path on the grid
//           TODO 8    grid_path()         the search that finds that path
// Do them in order: after TODO 1-6 EASY is solved, and the same driving code works for HARD.
// You don't need to know ROS: the robot's pose is in x_, y_, yaw_ and the goals in goals_.
// Everything at the bottom (marked "ROS") is done for you.
//
// Coordinates: metres, x to the right and y up (as in RViz), angles in radians, 0 = facing +x,
// positive = counter-clockwise (turning left). The robot starts at (0, 0) facing +x.
//
// Build and run it with (C++ needs a rebuild after every change):
//     colcon build --packages-select coderun_student
//     source install/setup.bash
//     ros2 run coderun_student solution_cpp
// Extra .cpp files you put in this src/ folder are compiled in too.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

// ---- settings you can change ----
constexpr bool USE_GRID_PATH = false;  // false = EASY (corner points, TODO 1), true = HARD (TODO 7 + 8)
constexpr double ARRIVED = 0.1;        // m: closer than this to a waypoint = "at the point"
constexpr double FACING = 0.1;         // rad (about 6 deg): turned less than this away = "facing it"

// ---- fixed ----
constexpr double MAX_LINEAR = 1.0;   // m/s, the simulator's limit: faster speeds are cut to this
constexpr double MAX_ANGULAR = 0.78; // rad/s, same for turning
constexpr double CELL = 0.5;         // planning grid cell size (m)
constexpr double CLEARANCE = 0.35;   // a cell is free when its centre is at least this far from any wall (m)

struct Goal
{
  double x, y, yaw;
};

struct Waypoint
{
  double x, y;
  std::optional<double> yaw;  // a goal's yaw, or std::nullopt for a point in between
};

struct Command
{
  double linear;   // forward speed, m/s
  double angular;  // turning speed, rad/s, + = left
  bool reached;    // true = done with this waypoint, the next one is used from now on
};

using Cell = std::pair<int, int>;  // (row, col) of the planning grid
using Grid = std::vector<std::vector<bool>>;

// Limit value to [-limit, +limit]. Example: clamp(2.0, 0.5) == 0.5.
double clamp(double value, double limit) { return std::max(-limit, std::min(limit, value)); }

// Wrap an angle to [-pi, pi], so that e.g. 350 deg becomes -10 deg (turn the short way).
double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Planning grid from /map: grid[row][col] is true when the robot can stand at the centre
// of that CELL x CELL square. Row 0 is the lowest y, column 0 the lowest x (like /map).
Grid build_grid(const nav_msgs::msg::OccupancyGrid & map)
{
  const int h = map.info.height, w = map.info.width;
  const double res = map.info.resolution;
  const int reach = static_cast<int>(std::ceil(CLEARANCE / res));
  const int rows = static_cast<int>(h * res / CELL), cols = static_cast<int>(w * res / CELL);
  Grid grid(rows, std::vector<bool>(cols, true));
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const int mr = static_cast<int>((r + 0.5) * CELL / res);  // /map cell under the centre
      const int mc = static_cast<int>((c + 0.5) * CELL / res);
      for (int dr = -reach; dr <= reach && grid[r][c]; ++dr) {
        for (int dc = -reach; dc <= reach; ++dc) {
          const int rr = mr + dr, cc = mc + dc;
          if (rr < 0 || rr >= h || cc < 0 || cc >= w || map.data[rr * w + cc] <= 50) continue;
          if (std::hypot(dr, dc) * res < CLEARANCE) {
            grid[r][c] = false;
            break;
          }
        }
      }
    }
  }
  return grid;
}

class SolutionNode : public rclcpp::Node
{
public:
  SolutionNode();

private:
  // ===================================================== STEP 1: plan the waypoints ==
  // Called once at the start. Returns the points to drive through, in order. Goals keep
  // their yaw (the robot must turn to it there), points in between get yaw = std::nullopt.
  std::vector<Waypoint> plan_waypoints()
  {
    std::vector<Waypoint> waypoints;
    double prev_x = x_, prev_y = y_;  // the robot starts here

    for (const auto & g : goals_) {  // the goals, in order
      if (!USE_GRID_PATH) {
        // EASY: drive first along x, then along y. That never hits a wall in EASY.
        // TODO 1: the corner point between (prev_x, prev_y) and the goal (g.x, g.y).
        //         Which x and which y does it have?
        double corner_x = g.x, corner_y = g.y;  // <- fix this (now it goes straight to the goal)
        waypoints.push_back({corner_x, corner_y, std::nullopt});
      } else {
        // HARD: the straight line is always blocked, so follow a path on the grid.
        const auto cells = grid_path(world_to_cell(prev_x, prev_y), world_to_cell(g.x, g.y));
        print_grid(cells);
        // TODO 7 (HARD): add the centre of every cell on the path as a waypoint (yaw std::nullopt).
        //         cell_to_world(row, col) gives the centre (x, y) of a cell.
        for (const auto & [row, col] : cells) {
          // <- your code here
        }
      }

      waypoints.push_back({g.x, g.y, g.yaw});  // the goal itself
      prev_x = g.x;                            // the next leg starts at this goal
      prev_y = g.y;
    }
    return waypoints;
  }

  // ================================================ STEP 2: drive to one waypoint ==
  // Called 10 times a second with the waypoint the robot is driving to.
  // Returns {linear, angular, reached}:
  //     linear    forward speed, m/s             (the simulator caps it at 1.0)
  //     angular   turning speed, rad/s, + = left (capped at 0.78)
  //     reached   true = done with this waypoint, the next one is used from now on
  Command follow_waypoint(const Waypoint & wp)
  {
    const double distance = distance_to(wp.x, wp.y);  // metres to the point
    const double angle = angle_to(wp.x, wp.y);        // how much to turn to face it (rad, + = left)

    if (distance > ARRIVED) {          // A. not there yet
      double linear, angular;
      if (std::abs(angle) > FACING) {  // A1. not facing the point: turn on the spot
        // TODO 2: stop (linear = 0) and turn towards the point.
        //         Hint: angular = a number * angle (the further off, the faster you turn)
        linear = 0.2;                  // <- fix this
        angular = 0.0;                 // <- fix this
      } else {                         // A2. facing it: drive
        // TODO 3: drive forward. Faster when far, slower when close (but at least 0.1 m/s).
        linear = 0.2;                  // <- try something with distance
        // TODO 4: small steering correction, so you keep facing the point.
        angular = 0.0;                 // <- try something with angle
      }
      return {linear, angular, false};
    }

    // B. at the point
    if (!wp.yaw) {                     // a point in between: nothing more to do
      return {0.0, 0.0, true};
    }

    const double error = heading_error(*wp.yaw);  // how much to turn to the goal yaw (rad, + = left)
    // TODO 5: turn on the spot towards the goal yaw (like TODO 2, with error).
    double angular = 0.0;              // <- fix this
    // TODO 6: reached when the heading is close enough. The judge allows 20 deg = 0.35 rad,
    //         but aim for much less (odometry is not perfect).
    bool reached = false;              // <- fix this
    return {0.0, angular, reached};
  }

  // ================================================== HARD: search the planning grid ==
  // Find a path on grid_ from cell start to cell goal (cells are (row, col)).
  // Return the cells of the path, start first, goal last: {start, ..., goal}.
  std::vector<Cell> grid_path(Cell start, Cell goal)
  {
    // TODO 8 (HARD): write a breadth-first search (BFS). The idea:
    //   - keep a queue of cells to look at, starting with start (std::queue<Cell>, <queue> is included)
    //   - keep a map came_from: cell -> the cell you reached it from (std::map<Cell, Cell>)
    //   - take the oldest cell from the queue; stop when it is the goal
    //   - for every cell in free_neighbours(cell) that is not in came_from yet (count() == 0):
    //     remember where you came from and add it to the queue
    //   - at the end, walk back from goal to start with came_from, and reverse that list
    //     (std::reverse, <algorithm> is included)
    // BFS finds the path with the fewest cells. Check your path with print_grid(path).
    return {start, goal};  // <- placeholder: straight to the goal (hits walls)
  }

  // =================================================================== helpers ==
  // Distance in metres from the robot to the point (x, y).
  double distance_to(double x, double y) const { return std::hypot(x - x_, y - y_); }

  // How much the robot has to turn to face the point (x, y): rad, + = left, - = right.
  double angle_to(double x, double y) const { return wrap(std::atan2(y - y_, x - x_) - yaw_); }

  // How much the robot has to turn to face the direction yaw: rad, + = left, - = right.
  double heading_error(double yaw) const { return wrap(yaw - yaw_); }

  // (x, y) in metres -> (row, col) of the planning grid cell that contains it.
  Cell world_to_cell(double x, double y) const
  {
    const auto & o = map_->info.origin.position;
    return {static_cast<int>(std::floor((y - o.y) / CELL)), static_cast<int>(std::floor((x - o.x) / CELL))};
  }

  // (row, col) of the planning grid -> (x, y) in metres of the cell centre.
  std::pair<double, double> cell_to_world(int row, int col) const
  {
    const auto & o = map_->info.origin.position;
    return {o.x + (col + 0.5) * CELL, o.y + (row + 0.5) * CELL};
  }

  // The free cells next to cell = (row, col): up, down, left and right (no diagonals).
  std::vector<Cell> free_neighbours(Cell cell) const
  {
    const auto [r, c] = cell;
    std::vector<Cell> out;
    for (const Cell & n : {Cell{r + 1, c}, Cell{r - 1, c}, Cell{r, c + 1}, Cell{r, c - 1}}) {
      if (n.first >= 0 && n.first < static_cast<int>(grid_.size()) && n.second >= 0 &&
        n.second < static_cast<int>(grid_[0].size()) && grid_[n.first][n.second])
      {
        out.push_back(n);
      }
    }
    return out;
  }

  // Print the grid, top row = highest y (as in RViz): '#' blocked, '.' free,
  // 'R' robot, '1'.. goals, '*' cells of `path`.
  void print_grid(const std::vector<Cell> & path = {}) const
  {
    std::map<Cell, char> marks;
    for (const auto & cell : path) marks[cell] = '*';
    marks[world_to_cell(x_, y_)] = 'R';
    for (size_t i = 0; i < goals_.size(); ++i) {
      marks[world_to_cell(goals_[i].x, goals_[i].y)] = static_cast<char>('0' + (i + 1) % 10);
    }
    for (int r = static_cast<int>(grid_.size()) - 1; r >= 0; --r) {
      for (int c = 0; c < static_cast<int>(grid_[r].size()); ++c) {
        const auto it = marks.find({r, c});
        std::putchar(it != marks.end() ? it->second : (grid_[r][c] ? '.' : '#'));
      }
      std::putchar('\n');
    }
    std::fflush(stdout);
  }

  // ============================================ ROS: you don't need to change this ==
  // Runs 10 times a second: calls your STEP 1 once, then your STEP 2 for the current waypoint.
  void control_loop()
  {
    if (!have_goals_ || !map_ || !have_odom_) {
      publish(0.0, 0.0);  // wait for the goals, the map and the robot pose
      return;
    }

    if (!have_waypoints_) {
      waypoints_ = plan_waypoints();
      have_waypoints_ = true;
      print_grid();
      RCLCPP_INFO(get_logger(), "%zu waypoints", waypoints_.size());
    }

    if (index_ >= waypoints_.size()) {
      publish(0.0, 0.0);  // all done: stop
      return;
    }

    Command cmd = follow_waypoint(waypoints_[index_]);
    if (cmd.reached) {
      ++index_;
      RCLCPP_INFO(get_logger(), "waypoint %zu/%zu reached", index_, waypoints_.size());
      cmd.linear = cmd.angular = 0.0;
    }
    publish(cmd.linear, cmd.angular);
  }

  void publish(double linear, double angular)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = clamp(linear, MAX_LINEAR);
    cmd.angular.z = clamp(angular, MAX_ANGULAR);
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Goal> goals_;  // in order
  bool have_goals_ = false;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;  // 0.04 m cells, 100 = wall
  Grid grid_;                                    // planning grid, see build_grid()
  sensor_msgs::msg::LaserScan::SharedPtr scan_;  // not needed, but there if you want it
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  bool have_odom_ = false;
  std::vector<Waypoint> waypoints_;  // from plan_waypoints()
  bool have_waypoints_ = false;
  size_t index_ = 0;                 // the waypoint the robot is driving to now
};

SolutionNode::SolutionNode()
: Node("solution_node")
{
  // /goals and /map are published once and latched: ask for transient-local durability
  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  goals_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    "goals", latched, [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
      if (!have_goals_) RCLCPP_INFO(get_logger(), "received %zu goals", msg->poses.size());
      goals_.clear();
      for (const auto & p : msg->poses) {
        goals_.push_back({p.position.x, p.position.y, yaw_from_quaternion(p.orientation)});
      }
      have_goals_ = true;
    });
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", latched, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      map_ = msg;
      grid_ = build_grid(*msg);
    });
  // scan_->ranges[i] is the distance at angle scan_->angle_min + i * scan_->angle_increment
  // (0 rad = straight ahead, positive = left)
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "base_scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {scan_ = msg;});
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      x_ = msg->pose.pose.position.x;
      y_ = msg->pose.pose.position.y;
      yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
      have_odom_ = true;
    });
  timer_ = create_wall_timer(100ms, [this] {control_loop();});  // 10 Hz
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SolutionNode>());
  rclcpp::shutdown();
  return 0;
}
