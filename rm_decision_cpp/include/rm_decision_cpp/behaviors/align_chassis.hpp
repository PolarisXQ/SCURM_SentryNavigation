#ifndef RM_DECISION_ALIGN_CHASSIS_HPP_
#define RM_DECISION_ALIGN_CHASSIS_HPP_
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
// for tf2::getYaw
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



using namespace BT;
namespace rm_decision
{
  class AlignChassis : public StatefulActionNode
  {
  public:
    AlignChassis(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener);

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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double angle_diff_map_chassis_;
    double min_angle_diff_,target_angle_;
    double angular_speed_;

  };
} // end namespace rm_decision

#endif