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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/robot_status.hpp>

#include <rm_base/crc.hpp>
#include <rm_base/buffer_processor_factory.hpp>
#include <rm_base/protocol_types.hpp>

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

TEST(RobotStatusProcessor, test_robot_pos_processor)
{
  ext_robot_status_t robot_status;
  robot_status.robot_id = 1;
  robot_status.robot_level = 1;
  robot_status.current_hp = 1;
  robot_status.max_hp = 1;
  robot_status.shooter_barrel_cooling_value = 1;
  robot_status.shooter_barrel_heat_limit = 1;
  robot_status.chassis_power_limit = 1;
  robot_status.power_management_gimbal_output = 1;
  robot_status.power_management_chassis_output = 1;
  robot_status.power_management_shooter_output = 1;
  rmoss_base::FixedPacket64 packet;
  packet.load_data(static_cast<uint8_t>(rm_base::RecvID::ROBOTSTATUS), 1);
  packet.load_data(robot_status, 2);
  packet.set_check_byte(rm_base::Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_robot_pos_frame");
  EXPECT_TRUE(
    rm_base::ProcessFactory::create(
      static_cast<uint8_t>(rm_base::RecvID::ROBOTSTATUS),
      node.get()));
  EXPECT_TRUE(
    rm_base::ProcessFactory::process_packet(
      static_cast<uint8_t>(rm_base::RecvID::
      ROBOTSTATUS), packet));
  rclcpp::shutdown();
}
