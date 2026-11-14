// CodeRun - your solution node in C++ (same as python/solution.py, pick either one).
//
// TASK: the judge gives you a list of goal poses (x, y, yaw). Make the robot reach them
// IN ORDER, as FAST as you can, without touching anything. A goal counts when the robot is
// within 0.3 m of it AND its heading is within 20 deg of the goal yaw.
//
// The goals are RANDOM at every run (in EASY only the last one, in HARD all of them), so you
// must read them at runtime - hardcoded positions will not work.
//
// You have:
//     /goals      geometry_msgs/PoseArray  the goal poses, in order (latched / transient local)
//     /map        nav_msgs/OccupancyGrid   the arena, 0.04 m cells, 100 = wall (latched)
//     /odom       nav_msgs/Odometry        robot pose estimate (starts at (0, 0, 0))
//     /base_scan  sensor_msgs/LaserScan    270 deg field of view, 1 deg steps, 5 m range
// and you publish:
//     /cmd_vel    geometry_msgs/Twist      linear.x <= 1.0 m/s, angular.z <= 0.78 rad/s
//
// /goals, /map and /odom are all in the same frame (the robot starts at its origin).
// /ground_truth is for the judge only - do not rely on it.
//
// Plan waypoints that lead from goal to goal, then drive through them:
//     EASY  driving first along x, then along y from goal to goal always works.
//     HARD  a straight line to the next goal is always blocked. This template turns /map into a
//           small planning grid for you: grid_[row][col] is true where the robot can drive
//           (0.5 m cells, walls already inflated), see build_grid(). Search it from the robot's
//           cell to the goal's cell (e.g. breadth-first search, 4 neighbours), then drive through
//           the centres of the cells on the path: the lines between neighbouring free cell
//           centres keep clear of the walls.
//           world_to_cell() / cell_to_world() convert between metres and grid cells, and
//           print_grid() shows the grid in the terminal.
//
// Build and run it with (C++ needs a rebuild after every change):
//     cd ~/ros_ws && colcon build --packages-select coderun_student && source install/setup.bash
//     ros2 run coderun_student solution_cpp
// Extra .cpp / .hpp files you put in this src/ folder are compiled in too.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

constexpr double CELL = 0.5;        // planning grid cell size (m)
constexpr double CLEARANCE = 0.35;  // a cell is free when its centre is at least this far from any wall (m)

struct Pose2D
{
  double x, y, yaw;  // metres, metres, radians
};
using Cell = std::pair<int, int>;  // (row, col) of the planning grid
using Grid = std::vector<std::vector<bool>>;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Wrap an angle to [-pi, pi].
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

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
  SolutionNode()
  : Node("solution_node")
  {
    // /goals and /map are published once and latched: ask for transient-local durability
    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    goals_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "goals", latched, [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {goals_callback(msg);});
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", latched, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {map_callback(msg);});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "base_scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {scan_ = msg;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {odom_callback(msg);});
    timer_ = create_wall_timer(100ms, [this] {control_loop();});  // 10 Hz
  }

private:
  void goals_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    goals_.clear();
    for (const auto & p : msg->poses) {
      goals_.push_back({p.position.x, p.position.y, yaw_from_quaternion(p.orientation)});
    }
    have_goals_ = true;
    RCLCPP_INFO(get_logger(), "received %zu goals", goals_.size());
  }

  void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // msg->data[row * msg->info.width + col] is 100 for a wall, 0 for free space.
    // Cell (row, col) covers x in msg->info.origin.position.x + col * msg->info.resolution (+ one cell)
    // and y likewise with row. Row 0 is the lowest y.
    map_ = msg;
    grid_ = build_grid(*msg);
  }

  // scan_->ranges[i] is the distance at angle scan_->angle_min + i * scan_->angle_increment
  // (0 rad = straight ahead, positive = counter-clockwise / left)

  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
  }

  // ------------------------------------------------------------- grid helpers --
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

  // ------------------------------------------------------------------ control --
  void control_loop()
  {
    geometry_msgs::msg::Twist cmd;
    if (!have_goals_ || !map_) {
      cmd_pub_->publish(cmd);  // wait for the goals and the map
      return;
    }

    if (!have_waypoints_) {
      print_grid();
      // TODO: turn goals_ into a list of waypoints: intermediate points that lead
      //       around the walls from one goal to the next (HARD: search grid_)
      waypoints_ = goals_;
      have_waypoints_ = true;
    }

    // TODO: compute the velocity that follows waypoints_
    //       (turn towards the next waypoint, drive, turn to the goal yaw, ...)
    cmd.linear.x = 0.2;  // placeholder: just drive forward
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Pose2D> goals_;  // in order
  bool have_goals_ = false;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  Grid grid_;  // planning grid, see build_grid()
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  std::vector<Pose2D> waypoints_;
  bool have_waypoints_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SolutionNode>());
  rclcpp::shutdown();
  return 0;
}
