#ifndef RM_DECISION_CONTROL_GIMBAL_HPP_
#define RM_DECISION_CONTROL_GIMBAL_HPP_
#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/msg/goal_status_array.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "rm_decision_cpp/custume_types.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace BT;
namespace rm_decision
{
  class ControlGimbal : public SyncActionNode
  {
  public:
    ControlGimbal(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener);

    NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr direction_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
    geometry_msgs::msg::PoseStamped goal_in_map_;
    double yaw_;
    double pitch_;

  };
} // end namespace rm_decision

#endif