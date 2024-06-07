#ifndef RM_DECISION_Topics2Blackboard_HPP_
#define RM_DECISION_Topics2Blackboard_HPP_

#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/exceptions.h"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>

#include "auto_aim_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/game_state.hpp"

#include "rm_decision_cpp/custume_types.hpp"

#include <memory>
#include <optional>
#include <chrono>
#include <functional>
#include <string>

using namespace BT;
namespace rm_decision
{
  class Topics2Blackboard : public SyncActionNode
  {
  public:
    Topics2Blackboard(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener);
    ~Topics2Blackboard() override = default;
    NodeStatus tick() override;
    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<rm_interfaces::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

    void game_state_callback_(const rm_interfaces::msg::GameState::SharedPtr msg);
    void target_callback_(const auto_aim_interfaces::msg::Target::SharedPtr msg);

    void check_subscriber_();

    std::optional<rm_interfaces::msg::GameState> game_state_;
    std::optional<auto_aim_interfaces::msg::Target> target_;

    geometry_msgs::msg::Point target_r_map_;
    geometry_msgs::msg::PoseStamped target_pose_;
    std::string target_armor_id_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped t;
    std::string to_frame_;
    std::string team_;
    double tracking_timeout_s_;
    rclcpp::Time last_tracking_time_;
  };
} // end namespace rm_decision
#endif