#include "rm_decision_cpp/behaviors/align_chassis.hpp"

using namespace BT;
namespace rm_decision
{
  AlignChassis::AlignChassis(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener)
  : StatefulActionNode(name, config), node_(node), tf_buffer_(tf_buffer), tf_listener_(tf_listener)
  {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  NodeStatus AlignChassis::onStart()
  {
    auto target_angle = getInput<double>("target_angle");
    if (!target_angle)
    {
      RCLCPP_ERROR(node_->get_logger(), "target_angle is not set");
      return NodeStatus::FAILURE;
    }
    else
    {
      target_angle_ = target_angle.value()/180*M_PI;
    }
    auto min_angle_diff = getInput<double>("min_angle_diff");
    if (!min_angle_diff)
    {
      RCLCPP_ERROR(node_->get_logger(), "min_angle_diff is not set");
      return NodeStatus::FAILURE;
    }
    else
    {
      min_angle_diff_ = min_angle_diff.value()/180*M_PI;
    }
    auto angular_speed = getInput<double>("angular_speed");
    if (!angular_speed)
    {
      RCLCPP_ERROR(node_->get_logger(), "angular_speed is not set");
      return NodeStatus::FAILURE;
    }
    else
    {
      angular_speed_ = angular_speed.value();
    }
    return NodeStatus::RUNNING;
  }

  NodeStatus AlignChassis::onRunning()
  {
    // get transform from map to chassis
    try
    {
      geometry_msgs::msg::TransformStamped map_chassis = tf_buffer_->lookupTransform("map", "chassis_link", tf2::TimePointZero);
      angle_diff_map_chassis_ = tf2::getYaw(map_chassis.transform.rotation);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Can't get transform from map to chassis: %s", ex.what());
      return NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "AlignChassis started");
    
    double err = target_angle_ - angle_diff_map_chassis_;
    if (err > M_PI)
    {
      err -= 2 * M_PI;
    }
    else if (err < -M_PI)
    {
      err += 2 * M_PI;
    }
    RCLCPP_INFO(node_->get_logger(),"target_angle_ %f .angle_diff_map_chassis_ %f",target_angle_,angle_diff_map_chassis_);
    if (fabs(err) < min_angle_diff_)
    {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.angular.z = 0;
      cmd_vel_pub_->publish(cmd_vel);
      RCLCPP_INFO(node_->get_logger(), "AlignChassis succeeded");
      return NodeStatus::SUCCESS;
    }
    else
    {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.angular.z = err > 0 ? angular_speed_ : -angular_speed_;
      cmd_vel_pub_->publish(cmd_vel);
      RCLCPP_INFO(node_->get_logger(), "AlignChassis running");
      return NodeStatus::RUNNING;
    }
  }

  void AlignChassis::onHalted()
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = 0;
    cmd_vel_pub_->publish(cmd_vel);
    RCLCPP_INFO(node_->get_logger(), "AlignChassis halted");
  }

  PortsList AlignChassis::providedPorts()
  {
    return {InputPort<double>("target_angle"),
            InputPort<double>("min_angle_diff"),
            InputPort<double>("angular_speed")};
  }

} // end namespace rm_decision