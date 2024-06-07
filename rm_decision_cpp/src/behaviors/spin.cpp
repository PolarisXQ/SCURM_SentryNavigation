#include "rm_decision_cpp/behaviors/spin.hpp"
using namespace BT;
namespace rm_decision
{

  Spin::Spin(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
      : SyncActionNode(name, config), node_(node)
  {
    chassis_type_pub_ = node_->create_publisher<std_msgs::msg::Int8>("/chassis_type", rclcpp::QoS(1).transient_local());
    slope_degree_sub_ = node_->create_subscription<std_msgs::msg::Float32>("/slope_degree", rclcpp::QoS(1), std::bind(&Spin::slope_degree_callback, this, std::placeholders::_1));
    node_->get_parameter_or("stop_spin_slope_degree_thre", stop_spin_slope_degree_thre_, 14.0);
    slope_degree_received_ = false;
  }

  NodeStatus Spin::tick()
  {
    // rclcpp::spin_some(node_);
    auto spin = getInput<bool>("spin");
    if (!spin)
    {
      RCLCPP_WARN(rclcpp::get_logger("SPIN"), "error reading port [spin]");
      spin_ = true;
    }
    else
    {
      spin_ = spin.value();
    }

    if (slope_degree_ > stop_spin_slope_degree_thre_ && slope_degree_received_)
    {
      std_msgs::msg::Int8 chassis_type;
      chassis_type.data = 2;
      chassis_type_pub_->publish(chassis_type);
      slope_degree_received_ = false;
      return NodeStatus::SUCCESS;
    }
    if (spin_)
    {
      std_msgs::msg::Int8 chassis_type;
      chassis_type.data = 4;
      chassis_type_pub_->publish(chassis_type);
    }
    else
    {
      std_msgs::msg::Int8 chassis_type;
      chassis_type.data = 2;
      chassis_type_pub_->publish(chassis_type);
    }
    return NodeStatus::SUCCESS;
  }

  PortsList Spin::providedPorts()
  {
    const char *description = "spin or not.";
    return {InputPort<bool>("spin", description)};
  }

  void Spin::slope_degree_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    slope_degree_ = msg->data;
    slope_degree_received_ = true;
    RCLCPP_DEBUG(rclcpp::get_logger("SPIN"), "slope_degree: %f", slope_degree_);
  }

} // end namespace rm_decision