#ifndef RM_DECISION_NAV2POSE_HPP_
#define RM_DECISION_NAV2POSE_HPP_
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



using namespace BT;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
namespace rm_decision
{
  class Nav2Pose : public StatefulActionNode
  {
  public:
    Nav2Pose(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

    // this function is invoked once at the beginning.
    NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    // action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    // goal handle
    NavigateToPoseGoalHandle::SharedPtr goal_handle_;
    // send goal timeout
    int send_goal_timeout_;
    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  };
} // end namespace rm_decision

#endif