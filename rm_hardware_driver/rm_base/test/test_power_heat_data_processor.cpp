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
#include <rm_interfaces/msg/power_heat_data.hpp>

#include <rm_base/crc.hpp>
#include <rm_base/buffer_processor_factory.hpp>
#include <rm_base/protocol_types.hpp>

typedef struct __packed
{
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t buffer_energy;
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
} __attribute__((__packed__)) ext_power_heat_data_t;

TEST(PowerHeatDataProcessor, test_power_heat_data_processor)
{
  ext_power_heat_data_t power_heat_data;
  power_heat_data.chassis_volt = 1;
  power_heat_data.chassis_current = 1;
  power_heat_data.chassis_power = 1.0;
  power_heat_data.buffer_energy = 1;
  power_heat_data.shooter_id1_17mm_cooling_heat = 40;
  power_heat_data.shooter_id2_17mm_cooling_heat = 40;
  power_heat_data.shooter_id1_42mm_cooling_heat = 40;
  rmoss_base::FixedPacket64 packet;
  packet.load_data(static_cast<uint8_t>(rm_base::RecvID::POWERHEATDATA), 1);
  packet.load_data(power_heat_data, 2);
  packet.set_check_byte(rm_base::Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_power_heat_data_frame");
  EXPECT_TRUE(
    rm_base::ProcessFactory::create(
      static_cast<uint8_t>(rm_base::RecvID::POWERHEATDATA),
      node.get()));
  EXPECT_TRUE(
    rm_base::ProcessFactory::process_packet(
      static_cast<uint8_t>(rm_base::RecvID::
      POWERHEATDATA), packet));
  rclcpp::shutdown();
}
