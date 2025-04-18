// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behavior_ext_plugins/back_up_twz_free_action.hpp"
#include <cmath>

namespace nav2_behaviors
{
  void BackUpTwzFree::onConfigure()
  {
    auto node = this->node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "robot_radius", rclcpp::ParameterValue(0.1));
    node->get_parameter("robot_radius", robot_radius_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "max_radius", rclcpp::ParameterValue(1.0));
    node->get_parameter("max_radius", max_radius_);

    if(max_radius_ < robot_radius_)
    {
      RCLCPP_WARN(node->get_logger(), "max_radius is smaller than robot_radius. Setting max_radius to robot_radius");
      max_radius_ = robot_radius_;
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "service_name", rclcpp::ParameterValue(std::string("local_costmap/get_costmap")));
    node->get_parameter("service_name", service_name_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "free_threshold", rclcpp::ParameterValue(5));
    node->get_parameter("free_threshold", free_threshold_);

    nav2_util::declare_parameter_if_not_declared(
      node,
      "visualization", rclcpp::ParameterValue(false));
    node->get_parameter("visualization", visualization_);
    
    costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("back_up_twz_free_markers", 1);

    RCLCPP_DEBUG(node->get_logger(), "back_up_twz_free_action plugin initialized.");
  }

  Status BackUpTwzFree::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
  {
    // send request to get costmap
    auto node = this->node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    while (!costmap_client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return Status::FAILED;
      }
      RCLCPP_WARN(node->get_logger(), "service not available, waiting again...");
    }

    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    auto result = costmap_client_->async_send_request(request);
    if(result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return Status::FAILED;
    }

    RCLCPP_DEBUG(node->get_logger(), "Got costmap");

    // get costmap
    auto costmap = result.get()->map;

    if (!nav2_util::getCurrentPose(
            initial_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }

    // move towards free space
    // get current pose
    auto pose_x = initial_pose_.pose.position.x;
    auto pose_y = initial_pose_.pose.position.y;
    auto yaw = tf2::getYaw(initial_pose_.pose.orientation);

    auto free_space_found = false;

    // // visualize costmap
    // cv::Mat img(costmap.metadata.size_y, costmap.metadata.size_x, CV_8UC1);
    // // init img
    // for (auto i = 0; i < costmap.metadata.size_x; i++)
    // {
    //   for (auto j = 0; j < costmap.metadata.size_y; j++)
    //   {
    //     img.at<uchar>(j, i) = 255; // set all pixels to white
    //   }
    // }
    // for (auto i = 0; i < costmap.metadata.size_x; i++)
    // {
    //   for (auto j = 0; j < costmap.metadata.size_y; j++)
    //   {
    //     auto costmap_index = i + j * costmap.metadata.size_x;
    //     img.at<uchar>(i,j) = costmap.data[costmap_index];
    //   }
    // }
    // cv::imshow("cost map", img);
    // cv::waitKey(5000);

    // expand circle until free space is found
    auto radius = robot_radius_;
    std::vector<geometry_msgs::msg::Point> free_points;
    while (!free_space_found && radius <= max_radius_)
    {
      // calculate sum of free space in circle
      int free_space_sum = 0;
      for (auto i = 0; i < costmap.metadata.size_x; i++)
      {
        for (auto j = 0; j < costmap.metadata.size_y; j++)
        {
          auto costmap_index = i + j * costmap.metadata.size_x;
          auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
          auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
          auto distance_to_center = std::hypot(x - pose_x, y - pose_y);
          if (distance_to_center <= radius)
          {
            if (costmap.data[costmap_index] == 0)
            {
              free_space_sum++;
              free_points.push_back(geometry_msgs::msg::Point());
              free_points.back().x = x;
              free_points.back().y = y;
            }
          }
        }
      }

      if (free_space_sum > free_threshold_)
      {
        free_space_found = true;
        RCLCPP_WARN(node->get_logger(), "free space found at radius: %f", radius);
        break;
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "free space not found at radius: %f", radius);
        radius += 0.1;
      }
    }

    // calculate avg position of free space
    auto avg_x = 0.0;
    auto avg_y = 0.0;
    for (auto i = 0; i < free_points.size(); i++)
    {
      avg_x += free_points[i].x;
      avg_y += free_points[i].y;
    }
    avg_x /= free_points.size();
    avg_y /= free_points.size();
    RCLCPP_WARN(node->get_logger(), "avg_x: %f, avg_y: %f", avg_x, avg_y);

    // visualize free space and destination
    if(visualization_){
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = node->now();
      marker.ns = "free_space";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = costmap.metadata.resolution;
      marker.scale.y = costmap.metadata.resolution;
      marker.color.r = 1.0;
      marker.color.a = 1.0;
      for (auto i = 0; i < free_points.size(); i++)
      {
        marker.points.push_back(free_points[i]);
      }
      markers.markers.push_back(marker);
      visualization_msgs::msg::Marker destination_marker;
      destination_marker.header.frame_id = global_frame_;
      destination_marker.header.stamp = node->now();
      destination_marker.ns = "destination";
      destination_marker.id = 0;
      destination_marker.type = visualization_msgs::msg::Marker::POINTS;
      destination_marker.action = visualization_msgs::msg::Marker::ADD;
      destination_marker.pose.orientation.w = 1.0;
      destination_marker.scale.x = costmap.metadata.resolution;
      destination_marker.scale.y = costmap.metadata.resolution;
      destination_marker.color.g = 1.0;
      destination_marker.color.a = 1.0;
      destination_marker.points.push_back(geometry_msgs::msg::Point());
      destination_marker.points.back().x = avg_x;
      destination_marker.points.back().y = avg_y;
      markers.markers.push_back(destination_marker);    
      marker_pub_->publish(markers);
    }
    
    // calculate angle to free space
    auto angle_to_free_space = std::atan2(avg_y - pose_y, avg_x - pose_x);
    auto angle_diff = angle_to_free_space - yaw;
    if (angle_diff > M_PI)
    {
      angle_diff -= 2 * M_PI;
    }
    else if (angle_diff < -M_PI)
    {
      angle_diff += 2 * M_PI;
    }
    RCLCPP_WARN(node->get_logger(), "angle_diff: %f deg", angle_diff*180/M_PI);

    // calculate move command
    twist_x_ = std::cos(angle_diff) * command->speed;
    twist_y_ = std::sin(angle_diff) * command->speed;
    command_x_ = command->target.x;
    command_time_allowance_ = command->time_allowance;

    end_time_ = this->clock_->now() + command_time_allowance_;

    if (!nav2_util::getCurrentPose(
            initial_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }
    RCLCPP_WARN(
        this->logger_, "backing up %f meters towards free space at angle %f", command_x_, angle_diff);

    return Status::SUCCEEDED;
  }

  Status BackUpTwzFree::onCycleUpdate()
  {
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0)
    {
      this->stopRobot();
      RCLCPP_WARN(
          this->logger_,
          "Exceeded time allowance before reaching the DriveOnHeading goal - Exiting DriveOnHeading");
      return Status::FAILED;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
            current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
            this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return Status::FAILED;
    }

    double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = hypot(diff_x, diff_y);

    feedback_->distance_traveled = distance;
    this->action_server_->publish_feedback(feedback_);

    if (distance >= std::fabs(command_x_))
    {
      this->stopRobot();
      return Status::SUCCEEDED;
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.y = twist_y_;
    cmd_vel->linear.x = twist_x_;

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = current_pose.pose.position.x;
    pose2d.y = current_pose.pose.position.y;
    pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

    if (!isCollisionFree(distance, cmd_vel.get(), pose2d))
    {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
      return Status::FAILED;
    }

    this->vel_pub_->publish(std::move(cmd_vel));

    return Status::RUNNING;
  }

} // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUpTwzFree, nav2_core::Behavior)