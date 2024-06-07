#ifndef RM_DECISION_SENTRY_CMD_HPP_
#define RM_DECISION_SENTRY_CMD_HPP_
#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "rm_interfaces/msg/sentry_cmd.hpp"
#include <rclcpp/qos.hpp>
#include <optional>

using namespace BT;
namespace rm_decision
{
  class SentryCmd : public SyncActionNode
  {
  public:
    SentryCmd(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~SentryCmd() override = default;
    // this function is invoked once at the beginning.
    NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    rm_interfaces::msg::SentryCmd sentry_cmd_;
    rclcpp::Publisher<rm_interfaces::msg::SentryCmd>::SharedPtr sentry_cmd_pub_;
  };
} // end namespace rm_decision
#endif