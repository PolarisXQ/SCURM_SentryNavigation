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
#include "rm_interfaces/msg/game_robot_hp.hpp"

namespace rm_base
{

typedef struct
{
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} __attribute__((__packed__)) ext_game_robot_hp_t;

class RobotHpProcessor : public ProcessInterface
{
public:
  explicit RobotHpProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("robot_hp_topic", "robot_hp");
    RCLCPP_INFO(node_->get_logger(), "robot_hp_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::GameRobotHp>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::GameRobotHp::UniquePtr msg(new rm_interfaces::msg::GameRobotHp());
      // msg->header.stamp = node_->now();

      ext_game_robot_hp_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->red_1_robot_hp = data.red_1_robot_HP;
      msg->red_2_robot_hp = data.red_2_robot_HP;
      msg->red_3_robot_hp = data.red_3_robot_HP;
      msg->red_4_robot_hp = data.red_4_robot_HP;
      msg->red_5_robot_hp = data.red_5_robot_HP;
      msg->red_7_robot_hp = data.red_7_robot_HP;
      msg->red_outpost_hp = data.red_outpost_HP;
      msg->red_base_hp = data.red_base_HP;
      msg->blue_1_robot_hp = data.blue_1_robot_HP;
      msg->blue_2_robot_hp = data.blue_2_robot_HP;
      msg->blue_3_robot_hp = data.blue_3_robot_HP;
      msg->blue_4_robot_hp = data.blue_4_robot_HP;
      msg->blue_5_robot_hp = data.blue_5_robot_HP;
      msg->blue_7_robot_hp = data.blue_7_robot_HP;
      msg->blue_outpost_hp = data.blue_outpost_HP;
      msg->blue_base_hp = data.blue_outpost_HP;

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for GameRobotHp processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::GameRobotHp>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(RobotHpProcessor, static_cast<uint8_t>(RecvID::ROBOTHP))

}  // namespace rm_base
