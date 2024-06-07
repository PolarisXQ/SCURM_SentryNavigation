#ifndef RM_DECISION_SPIN_HPP_
#define RM_DECISION_SPIN_HPP_
#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include "auto_aim_interfaces/msg/target.hpp"
#include <rclcpp/qos.hpp>
#include <optional>

using namespace BT;
namespace rm_decision
{
  class Spin : public SyncActionNode
  {
  public:
    Spin(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~Spin() override = default;
    // this function is invoked once at the beginning.
    NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    bool spin_;
    bool slope_degree_received_;
    double stop_spin_slope_degree_thre_;
    double slope_degree_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr chassis_type_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr slope_degree_sub_;

    void slope_degree_callback(const std_msgs::msg::Float32::SharedPtr msg);
  };
} // end namespace rm_decision
#endif