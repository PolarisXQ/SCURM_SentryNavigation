#include "rm_decision_cpp/behaviors/sentry_cmd.hpp"
using namespace BT;
namespace rm_decision
{

  SentryCmd::SentryCmd(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
  : SyncActionNode(name, config),node_(node)
  {
    sentry_cmd_pub_ = node_->create_publisher<rm_interfaces::msg::SentryCmd>("/sentry_cmd", rclcpp::QoS(1).transient_local());
  }

  NodeStatus SentryCmd::tick()
  {
    sentry_cmd_.resurrection_en = true;
    sentry_cmd_.buy_resurrection_en = false;
    sentry_cmd_.buy_projectile_allowance = 0;
    sentry_cmd_.buy_projectile_times = 0;
    sentry_cmd_.buy_hp_times= 0;

    sentry_cmd_pub_->publish(sentry_cmd_);
    
    return NodeStatus::SUCCESS;
  }

  PortsList SentryCmd::providedPorts()
  {
    return {
      InputPort<bool>("resurrection_en"),
      InputPort<bool>("buy_resurrection_en"),
      InputPort<std::uint16_t>("buy_projectile_allowance"),
      InputPort<std::uint8_t>("buy_projectile_times"),
      InputPort<std::uint8_t>("buy_hp_times")
    };
  }

} // end namespace rm_decision