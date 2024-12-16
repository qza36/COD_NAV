#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

using namespace std::chrono_literals;

class ClearCostmapCaller : public rclcpp::Node
{
public:
  ClearCostmapCaller()
  : Node("clear_costmap_caller")
  {
    // 创建全局和局部成本地图服务客户端
    global_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entirely_global_costmap");
    local_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/local_costmap/clear_entirely_local_costmap");

    RCLCPP_INFO(this->get_logger(), "Waiting for services to become available...");

    // 等待服务可用
    while (!global_client_->wait_for_service(1s) || !local_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Services not available, waiting...");
    }

    RCLCPP_INFO(this->get_logger(), "Services available, starting timer.");

    // 创建定时器，每0.5秒调用一次服务
    timer_ = this->create_wall_timer(1000ms, std::bind(&ClearCostmapCaller::call_services, this));
  }

private:
  void call_services()
  {
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

    // 异步调用全局成本地图清理服务
    auto global_future_result = global_client_->async_send_request(
      request, [this](const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future) {
        this->service_response_callback("Global Costmap", future);
      });

    // 异步调用局部成本地图清理服务
    auto local_future_result = local_client_->async_send_request(
      request, [this](const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future) {
        this->service_response_callback("Local Costmap", future);
      });

    RCLCPP_INFO(this->get_logger(), "Service calls sent for clearing costmaps.");
  }

  void service_response_callback(
    const std::string & costmap_type,
    const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture & future)
  {
    try {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Successfully cleared %s.", costmap_type.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to clear %s: %s", costmap_type.c_str(), e.what());
    }
  }

  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClearCostmapCaller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
