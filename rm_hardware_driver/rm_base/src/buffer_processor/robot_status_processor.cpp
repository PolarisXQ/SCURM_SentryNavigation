// Copyright 2023 Tingxu Chen
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

#include "rm_base/buffer_processor_factory.hpp"

#include <rm_base/protocol_types.hpp>
#include "rm_interfaces/msg/robot_status.hpp"

namespace rm_base
{

typedef struct __packed
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_hp;
  uint16_t max_hp;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  uint8_t power_management_gimbal_output;
  uint8_t power_management_chassis_output;
  uint8_t power_management_shooter_output;
} __attribute__((__packed__)) ext_robot_status_t;

class RobotStatusProcessor : public ProcessInterface
{
public:
  explicit RobotStatusProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("robot_status_topic", "robot_status");
    RCLCPP_INFO(node_->get_logger(), "robot_status_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::RobotStatus>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::RobotStatus::UniquePtr msg(new rm_interfaces::msg::RobotStatus());

      ext_robot_status_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->robot_id = data.robot_id;
      msg->robot_level = data.robot_level;
      msg->current_hp = data.current_hp;
      msg->max_hp = data.max_hp;
      msg->shooter_barrel_cooling_value = data.shooter_barrel_cooling_value;
      msg->shooter_barrel_heat_limit = data.shooter_barrel_heat_limit;
      msg->chassis_power_limit = data.chassis_power_limit;
      msg->power_management_gimbal_output = data.power_management_gimbal_output;
      msg->power_management_chassis_output = data.power_management_chassis_output;
      msg->power_management_shooter_output = data.power_management_shooter_output;

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for RobotStatus processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::RobotStatus>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(RobotStatusProcessor, static_cast<uint8_t>(RecvID::ROBOTSTATUS))

}  // namespace rm_base
