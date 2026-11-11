#include "velocity_manager/velocity_manager_node.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace velocity_manager
{

VelocityManagerNode::VelocityManagerNode(const rclcpp::NodeOptions & options)
: Node("velocity_manager", options)
{
  declareParameters();
  loadParameters();
  setupInterfaces();

  RCLCPP_INFO(
    get_logger(),
    "velocity_manager ready: output='%s', selected='%s', enabled=%s, sources=%zu",
    output_topic_.c_str(),
    inputs_[current_index_].name.c_str(),
    enabled_ ? "true" : "false",
    inputs_.size());
}

void VelocityManagerNode::declareParameters()
{
  declare_parameter<std::string>("output_topic", "cmd_vel");
  declare_parameter<std::string>("default_input", "");
  declare_parameter<std::vector<std::string>>("input_names", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("input_topics", std::vector<std::string>{});
}

void VelocityManagerNode::loadParameters()
{
  output_topic_ = get_parameter("output_topic").as_string();
  default_input_ = get_parameter("default_input").as_string();

  const auto names = get_parameter("input_names").as_string_array();
  const auto topics = get_parameter("input_topics").as_string_array();

  if (names.empty()) {
    throw std::runtime_error("Parameter 'input_names' must not be empty");
  }
  if (names.size() != topics.size()) {
    throw std::runtime_error(
      "Parameters 'input_names' and 'input_topics' must have the same length");
  }

  inputs_.clear();
  inputs_.reserve(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i].empty()) {
      throw std::runtime_error("input_names contains an empty name");
    }
    if (topics[i].empty()) {
      throw std::runtime_error("input_topics contains an empty topic for source '" + names[i] + "'");
    }
    const auto duplicate = std::find_if(
      inputs_.begin(), inputs_.end(),
      [&](const InputSource & src) { return src.name == names[i]; });
    if (duplicate != inputs_.end()) {
      throw std::runtime_error("Duplicate input source name: '" + names[i] + "'");
    }

    InputSource source;
    source.name = names[i];
    source.topic = topics[i];
    inputs_.push_back(std::move(source));
  }

  const auto it = std::find_if(
    inputs_.begin(), inputs_.end(),
    [this](const InputSource & src) { return src.name == default_input_; });
  if (it == inputs_.end()) {
    throw std::runtime_error(
      "default_input '" + default_input_ + "' was not found in input_names");
  }
  current_index_ = static_cast<std::size_t>(std::distance(inputs_.begin(), it));
  enabled_ = false;
}

void VelocityManagerNode::setupInterfaces()
{
  output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

  // Latched-like status so late subscribers receive the latest enable state.
  const auto enabled_qos = rclcpp::QoS(10).transient_local();
  enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/vel_mng/enabled", enabled_qos);
  publishEnabledState();

  for (std::size_t i = 0; i < inputs_.size(); ++i) {
    inputs_[i].subscription = create_subscription<geometry_msgs::msg::Twist>(
      inputs_[i].topic, 10,
      [this, i](const geometry_msgs::msg::Twist::SharedPtr msg) {
        onInputTwist(i, msg);
      });
    RCLCPP_INFO(
      get_logger(), "Subscribed input[%zu]: name='%s' topic='%s'",
      i, inputs_[i].name.c_str(), inputs_[i].topic.c_str());
  }

  switch_srv_ = create_service<std_srvs::srv::Trigger>(
    "/vel_mng/switch",
    std::bind(
      &VelocityManagerNode::handleSwitch, this,
      std::placeholders::_1, std::placeholders::_2));

  enable_srv_ = create_service<std_srvs::srv::SetBool>(
    "/vel_mng/enable",
    std::bind(
      &VelocityManagerNode::handleEnable, this,
      std::placeholders::_1, std::placeholders::_2));
}

void VelocityManagerNode::onInputTwist(
  const std::size_t source_index,
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }
  if (source_index != current_index_) {
    return;
  }
  output_pub_->publish(*msg);
}

void VelocityManagerNode::handleSwitch(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  current_index_ = (current_index_ + 1) % inputs_.size();
  response->success = true;
  response->message = inputs_[current_index_].name;

  RCLCPP_INFO(
    get_logger(), "Switched source to '%s' (enabled=%s)",
    inputs_[current_index_].name.c_str(),
    enabled_ ? "true" : "false");
}

void VelocityManagerNode::handleEnable(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!request->data) {
    enabled_ = false;
    publishZeroTwist();
    publishEnabledState();
    response->success = true;
    response->message = "disabled";
    RCLCPP_INFO(
      get_logger(), "Output disabled; published zero Twist (selected='%s')",
      inputs_[current_index_].name.c_str());
    return;
  }

  enabled_ = true;
  publishEnabledState();
  response->success = true;
  response->message = "enabled:" + inputs_[current_index_].name;
  RCLCPP_INFO(
    get_logger(), "Output enabled with source '%s'",
    inputs_[current_index_].name.c_str());
}

void VelocityManagerNode::publishZeroTwist()
{
  geometry_msgs::msg::Twist zero;
  output_pub_->publish(zero);
}

void VelocityManagerNode::publishEnabledState()
{
  std_msgs::msg::Bool msg;
  msg.data = enabled_;
  enabled_pub_->publish(msg);
}

}  // namespace velocity_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<velocity_manager::VelocityManagerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    fprintf(stderr, "Failed to start velocity_manager: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
