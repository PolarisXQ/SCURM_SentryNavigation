#include "rm_decision_cpp/behaviors/control_gimbal.hpp"

using namespace BT;
namespace rm_decision
{
  ControlGimbal::ControlGimbal(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener)
  : SyncActionNode(name, config), node_(node), tf_buffer_(tf_buffer), tf_listener_(tf_listener)
  {
    direction_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("direction_to_goal", 10);
    armors_pub_ = node_->create_publisher<auto_aim_interfaces::msg::Armors>("/detector/navarmors", 10);
    RCLCPP_INFO(node_->get_logger(), "ControlGimbal initialized");
    // TODO: disable when full detect is working
  }

  NodeStatus ControlGimbal::tick()
  {
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal)
    {
      RCLCPP_WARN(node_->get_logger(), "goal is not set");
    }
    goal_in_map_ = goal.value();
    goal_in_map_.pose.orientation.x = 0;
    goal_in_map_.pose.orientation.y = 0;
    goal_in_map_.pose.orientation.z = 0;
    goal_in_map_.pose.orientation.w = 1;
    // get position of gimbal_odom in map
    geometry_msgs::msg::TransformStamped gimbal_odom_in_map;
    try
    {
      gimbal_odom_in_map = tf_buffer_->lookupTransform("gimbal_odom","map",tf2::TimePointZero);
      RCLCPP_DEBUG(node_->get_logger(), "x: %f, y: %f, z: %f", gimbal_odom_in_map.transform.translation.x, gimbal_odom_in_map.transform.translation.y, gimbal_odom_in_map.transform.translation.z);
      RCLCPP_DEBUG(node_->get_logger(), "qx: %f, qy: %f, qz: %f, qw: %f", gimbal_odom_in_map.transform.rotation.x, gimbal_odom_in_map.transform.rotation.y, gimbal_odom_in_map.transform.rotation.z, gimbal_odom_in_map.transform.rotation.w);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Can't get transform from map to gimbal_odom: %s", ex.what());
      return NodeStatus::FAILURE;
    }
    // calculate goal in gimbal_odom
    geometry_msgs::msg::PoseStamped goal_in_gimbal_odom;
    tf2::doTransform(goal_in_map_, goal_in_gimbal_odom, gimbal_odom_in_map);
    RCLCPP_DEBUG(node_->get_logger(), "goal in gimbal_odom: x: %f, y: %f, z: %f", goal_in_gimbal_odom.pose.position.x, goal_in_gimbal_odom.pose.position.y, goal_in_gimbal_odom.pose.position.z);
    RCLCPP_DEBUG(node_->get_logger(), "goal in gimbal_odom: qx: %f, qy: %f, qz: %f, qw: %f", goal_in_gimbal_odom.pose.orientation.x, goal_in_gimbal_odom.pose.orientation.y, goal_in_gimbal_odom.pose.orientation.z, goal_in_gimbal_odom.pose.orientation.w);
    // publish direction by an arrow from gimbal_odom to goal
    visualization_msgs::msg::MarkerArray direction;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "gimbal_odom";
    marker.header.stamp = node_->now();
    marker.ns = "direction";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    double yaw = atan2(goal_in_gimbal_odom.pose.position.y, goal_in_gimbal_odom.pose.position.x);
    double pitch = atan2(goal_in_gimbal_odom.pose.position.z, sqrt(pow(goal_in_gimbal_odom.pose.position.x, 2) + pow(goal_in_gimbal_odom.pose.position.y, 2)));
    tf2::Quaternion q;
    q.setRPY(0, -pitch, yaw);
    marker.pose.orientation = tf2::toMsg(q);
    marker.scale.x = sqrt(pow(goal_in_gimbal_odom.pose.position.x, 2) + pow(goal_in_gimbal_odom.pose.position.y, 2) + pow(goal_in_gimbal_odom.pose.position.z, 2));
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    direction.markers.push_back(marker);
    direction_pub_->publish(direction);
    // publish armors
    auto armors = std::make_unique<auto_aim_interfaces::msg::Armors>();
    armors->header.stamp = node_->now();
    armors->header.frame_id = "gimbal_odom";
    armors->fulldetect = true;
    auto armor = std::make_unique<auto_aim_interfaces::msg::Armor>();
    armor->header.stamp = node_->now();
    armor->header.frame_id = "gimbal_odom";
    armor->number = "outpost";
    armor->pose = goal_in_gimbal_odom.pose;
    armors->armors.push_back(*armor);
    armors_pub_->publish(std::move(armors));
    return NodeStatus::SUCCESS;
  }

  PortsList ControlGimbal::providedPorts()
  {
    const char *goal_description = "goal pose in map frame.";
    return {InputPort<geometry_msgs::msg::PoseStamped>("goal", goal_description)};
  }

} // end namespace rm_decision