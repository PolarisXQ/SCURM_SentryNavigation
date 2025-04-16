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
#include <rm_interfaces/msg/game_robot_hp.hpp>

#include <rm_base/crc.hpp>
#include <rm_base/buffer_processor_factory.hpp>
#include <rm_base/protocol_types.hpp>

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

TEST(RobotHpProcessor, test_game_robot_hp_processor)
{
  ext_game_robot_hp_t robot_hp;
  robot_hp.red_1_robot_HP = 100;
  robot_hp.red_2_robot_HP = 100;
  robot_hp.red_3_robot_HP = 100;
  robot_hp.red_4_robot_HP = 100;
  robot_hp.red_5_robot_HP = 100;
  robot_hp.red_7_robot_HP = 100;
  robot_hp.red_outpost_HP = 100;
  robot_hp.red_base_HP = 100;
  robot_hp.blue_1_robot_HP = 100;
  robot_hp.blue_2_robot_HP = 100;
  robot_hp.blue_3_robot_HP = 100;
  robot_hp.blue_4_robot_HP = 100;
  robot_hp.blue_5_robot_HP = 100;
  robot_hp.blue_7_robot_HP = 100;
  robot_hp.blue_outpost_HP = 100;
  robot_hp.blue_base_HP = 100;
  rmoss_base::FixedPacket64 packet;
  packet.load_data(static_cast<uint8_t>(rm_base::RecvID::ROBOTHP), 1);
  packet.load_data(robot_hp, 2);
  packet.set_check_byte(rm_base::Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_game_robot_hp_frame");
  EXPECT_TRUE(
    rm_base::ProcessFactory::create(
      static_cast<uint8_t>(rm_base::RecvID::ROBOTHP),
      node.get()));
  EXPECT_TRUE(
    rm_base::ProcessFactory::process_packet(
      static_cast<uint8_t>(rm_base::RecvID::ROBOTHP),
      packet));
  rclcpp::shutdown();
}
