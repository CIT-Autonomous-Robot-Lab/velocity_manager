#ifndef VELOCITY_MANAGER__VELOCITY_MANAGER_NODE_HPP_
#define VELOCITY_MANAGER__VELOCITY_MANAGER_NODE_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace velocity_manager
{

class VelocityManagerNode : public rclcpp::Node
{
public:
  explicit VelocityManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct InputSource
  {
    std::string name;
    std::string topic;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
  };

  void declareParameters();
  void loadParameters();
  void setupInterfaces();

  void onInputTwist(std::size_t source_index, const geometry_msgs::msg::Twist::SharedPtr msg);

  void handleSwitch(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void publishZeroTwist();
  void publishEnabledState();

  std::string output_topic_;
  std::string default_input_;
  std::vector<InputSource> inputs_;
  std::size_t current_index_{0};
  bool enabled_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
};

}  // namespace velocity_manager

#endif  // VELOCITY_MANAGER__VELOCITY_MANAGER_NODE_HPP_
