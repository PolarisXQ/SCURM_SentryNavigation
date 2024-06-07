#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/json_export.h"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>


namespace BT
{
using StringView = std::string_view;

template <> [[nodiscard]]
geometry_msgs::msg::PointStamped convertFromString<geometry_msgs::msg::PointStamped>(StringView str);

inline void PointStampedToJson(nlohmann::json& dest, const geometry_msgs::msg::PointStamped& point)
{
  dest["x"] = point.point.x;
  dest["y"] = point.point.y;
  dest["z"] = point.point.z;
}

template <> [[nodiscard]]
geometry_msgs::msg::PoseStamped convertFromString<geometry_msgs::msg::PoseStamped>(StringView str);

inline void PoseStampedToJson(nlohmann::json& dest, const geometry_msgs::msg::PoseStamped& pose)
{
  dest["x"] = pose.pose.position.x;
  dest["y"] = pose.pose.position.y;
  dest["z"] = pose.pose.position.z;
  dest["qx"] = pose.pose.orientation.x;
  dest["qy"] = pose.pose.orientation.y;
  dest["qz"] = pose.pose.orientation.z;
  dest["qw"] = pose.pose.orientation.w;
}

}   // namespace BT

