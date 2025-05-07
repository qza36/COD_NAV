#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>

class ObstacleEscapeNode : public rclcpp::Node
{
public:
  ObstacleEscapeNode() : Node("obstacle_escape_node"), escaping_(false)
  {
    // Declare parameters
    this->declare_parameter("robot_radius", 0.2);
    this->declare_parameter("obstacle_threshold", 80);
    this->declare_parameter("escape_velocity", 0.2);
    this->declare_parameter("escape_duration", 1.0);

    // Get parameter values
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
    escape_velocity_ = this->get_parameter("escape_velocity").as_double();
    escape_duration_ = this->get_parameter("escape_duration").as_double();

    // Create subscription to costmap
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "local_costmap/costmap", 10,
      std::bind(&ObstacleEscapeNode::costmapCallback, this, std::placeholders::_1));

    // Create publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Obstacle Escape Node initialized");
  }

private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (escaping_) {
      return;
    }

    int width = msg->info.width;
    int height = msg->info.height;
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;

    // Get robot position in costmap coordinates (assuming robot at center of local costmap)
    int robot_x = static_cast<int>((0.0 - origin_x) / resolution);
    int robot_y = static_cast<int>((0.0 - origin_y) / resolution);

    // Calculate radius in grid cells
    int radius_cells = static_cast<int>(robot_radius_ / resolution);

    // Check for obstacles in robot footprint
    bool collision = checkCollision(msg->data, width, height, robot_x, robot_y, radius_cells);

    if (collision) {
      RCLCPP_INFO(this->get_logger(), "Collision detected! Attempting to escape...");
      auto escape_direction = determineEscapeDirection(msg->data, width, height, robot_x, robot_y, radius_cells);
      initiateEscape(escape_direction);
    }
  }

  bool checkCollision(const std::vector<int8_t>& costmap, int width, int height,
                     int robot_x, int robot_y, int radius_cells)
  {
    for (int y = std::max(0, robot_y - radius_cells); y <= std::min(height - 1, robot_y + radius_cells); ++y) {
      for (int x = std::max(0, robot_x - radius_cells); x <= std::min(width - 1, robot_x + radius_cells); ++x) {
        int dist_sq = (x - robot_x) * (x - robot_x) + (y - robot_y) * (y - robot_y);
        if (dist_sq <= radius_cells * radius_cells) {
          int index = y * width + x;
          if (index < costmap.size() && costmap[index] >= obstacle_threshold_) {
            return true;
          }
        }
      }
    }
    return false;
  }

  std::pair<double, double> determineEscapeDirection(const std::vector<int8_t>& costmap,
                                                   int width, int height,
                                                   int robot_x, int robot_y,
                                                   int radius_cells)
  {
    double dir_x = 0.0;
    double dir_y = 0.0;
    int count = 0;

    for (int y = std::max(0, robot_y - radius_cells); y <= std::min(height - 1, robot_y + radius_cells); ++y) {
      for (int x = std::max(0, robot_x - radius_cells); x <= std::min(width - 1, robot_x + radius_cells); ++x) {
        int dist_sq = (x - robot_x) * (x - robot_x) + (y - robot_y) * (y - robot_y);
        if (dist_sq <= radius_cells * radius_cells) {
          int index = y * width + x;
          if (index < costmap.size() && costmap[index] >= obstacle_threshold_) {
            // Move away from obstacle
            dir_x += static_cast<double>(robot_x - x);
            dir_y += static_cast<double>(robot_y - y);
            count++;
          }
        }
      }
    }

    if (count == 0) {
      // If no obstacle cells found in footprint, use the last escape direction
      // or a default direction (backward)
      if (last_escape_direction_.first != 0.0 || last_escape_direction_.second != 0.0) {
        return last_escape_direction_;
      } else {
        return {0.0, -1.0}; // Move backward by default
      }
    }

    // Normalize direction
    double magnitude = std::sqrt(dir_x * dir_x + dir_y * dir_y);
    dir_x /= magnitude;
    dir_y /= magnitude;

    last_escape_direction_ = {dir_x, dir_y};
    return last_escape_direction_;
  }

  void initiateEscape(const std::pair<double, double>& direction)
  {
    // Convert map direction to robot frame (assuming local costmap aligned with robot)
    // In ROS, x is forward, y is left
    double linear_x = direction.second;  // Forward/backward is y in grid coordinates
    double linear_y = -direction.first;  // Left/right is negative x in grid coordinates

    // Create Twist message
    auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
    cmd->linear.x = linear_x * escape_velocity_;
    cmd->linear.y = linear_y * escape_velocity_;
    cmd->angular.z = 0.0;

    // Publish velocity command
    cmd_vel_pub_->publish(*cmd);
    RCLCPP_INFO(this->get_logger(), "Escaping with velocity: [%f, %f]", cmd->linear.x, cmd->linear.y);

    // Set escaping flag and start timer
    escaping_ = true;
    escape_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(escape_duration_ * 1000)),
      std::bind(&ObstacleEscapeNode::stopEscape, this));
  }

  void stopEscape()
  {
    // Stop robot
    auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel_pub_->publish(*cmd);

    // Reset state
    escaping_ = false;
    escape_timer_.reset();

    RCLCPP_INFO(this->get_logger(), "Escape maneuver completed");
  }

  // Subscribers and publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr escape_timer_;

  // Parameters
  double robot_radius_;
  int obstacle_threshold_;
  double escape_velocity_;
  double escape_duration_;

  // State variables
  bool escaping_;
  std::pair<double, double> last_escape_direction_{0.0, 0.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleEscapeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}