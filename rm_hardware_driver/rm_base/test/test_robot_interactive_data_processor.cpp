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
#include <rm_interfaces/msg/robot_interactive_data.hpp>

#include <rm_base/crc.hpp>
#include <rm_base/buffer_processor_factory.hpp>
#include <rm_base/protocol_types.hpp>

typedef struct __packed
{
  uint16_t data_cmd_id;
  uint16_t send_id;
  uint16_t recv_id;
  uint16_t data[113];  // 128字节帧限制最大数据量 113 字节
} __attribute__((__packed__)) robot_interact_t;

TEST(RobotInteractiveDataProcessor, test_robot_interactive_data_processor)
{
  robot_interact_t referee_interact;
  referee_interact.data_cmd_id = 1;
  referee_interact.send_id = 1;
  referee_interact.recv_id = 1;
  for (int i = 0; i < 49; i++) {
    referee_interact.data[i] = i;
  }
  rmoss_base::FixedPacket64 packet;
  packet.load_data(static_cast<uint8_t>(rm_base::RecvID::ROBOTINTERACT), 1);
  packet.load_data(referee_interact, 2);
  packet.set_check_byte(rm_base::Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_robot_interactive_data_frame");
  EXPECT_TRUE(
    rm_base::ProcessFactory::create(
      static_cast<uint8_t>(rm_base::RecvID::ROBOTINTERACT),
      node.get()));
  EXPECT_TRUE(
    rm_base::ProcessFactory::process_packet(
      static_cast<uint8_t>(rm_base::RecvID::
      ROBOTINTERACT), packet));
  rclcpp::shutdown();
}
