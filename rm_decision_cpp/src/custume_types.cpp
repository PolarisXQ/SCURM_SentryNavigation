#include "rm_decision_cpp/custume_types.hpp"
#include "rclcpp/rclcpp.hpp"
namespace BT
{

  template <>
  geometry_msgs::msg::PointStamped convertFromString<geometry_msgs::msg::PointStamped>(StringView str)
  {
    const auto parts = BT::splitString(str, ',');
    if (parts.size() != 3)
    {
      throw BT::RuntimeError("invalid input");
    }

    geometry_msgs::msg::PointStamped point;
    point.point.x = convertFromString<double>(parts[0]);
    point.point.y = convertFromString<double>(parts[1]);
    point.point.z = convertFromString<double>(parts[2]);
    return point;
  }

  template <>
  geometry_msgs::msg::PoseStamped convertFromString<geometry_msgs::msg::PoseStamped>(StringView str)
  {
    const auto parts = BT::splitString(str, ',');
    if (parts.size() != 7)
    {
      throw BT::RuntimeError("invalid input");
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = convertFromString<double>(parts[0]);
    pose.pose.position.y = convertFromString<double>(parts[1]);
    pose.pose.position.z = convertFromString<double>(parts[2]);
    pose.pose.orientation.x = convertFromString<double>(parts[3]);
    pose.pose.orientation.y = convertFromString<double>(parts[4]);
    pose.pose.orientation.z = convertFromString<double>(parts[5]);
    pose.pose.orientation.w = convertFromString<double>(parts[6]);
    return pose;
  }
} // namespace BT