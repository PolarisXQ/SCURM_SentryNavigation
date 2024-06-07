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

#ifndef BEHAVIORS_EXT__PLUGINS__BACK_UP_HPP_
#define BEHAVIORS_EXT__PLUGINS__BACK_UP_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "visualization_msgs/msg/marker_array.hpp" 

using BackUpAction = nav2_msgs::action::BackUp;


namespace nav2_behaviors
{
class BackUpTwzFree : public DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
  Status onCycleUpdate();

protected:
  void onConfigure() override;

private:
  // client to get local costmap  
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string service_name_;
  double twist_x_, twist_y_;
  double max_radius_, robot_radius_;
  int free_threshold_;
  bool visualization_;

  // nav_msgs::msg::Odometry::SharedPtr odom_;
};
}

#endif  // BEHAVIORS_EXT__PLUGINS__BACK_UP_HPP_