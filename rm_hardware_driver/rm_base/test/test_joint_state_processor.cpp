// Copyright 2023 Yunlong Feng
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

#include <rm_base/crc.hpp>
#include <rm_base/buffer_processor_factory.hpp>
#include <rm_base/protocol_types.hpp>

typedef struct
{
  float yaw_position;
  float yaw_velocity;
  float pitch_position;
  float pitch_velocity;
  float wheel_left_front_position;
  float wheel_left_front_velocity;
  float wheel_left_rear_position;
  float wheel_left_rear_velocity;
  float wheel_right_front_position;
  float wheel_right_front_velocity;
  float wheel_right_rear_position;
  float wheel_right_rear_velocity;
} __attribute__((__packed__)) robot_joint_state_t;

TEST(JointStateProcessor, test_joint_state_processor)
{
  robot_joint_state_t joint_state_data;
  rmoss_base::FixedPacket64 packet;
  packet.load_data(static_cast<uint8_t>(rm_base::RecvID::ROBOTJOINTSTATE), 1);
  packet.load_data(joint_state_data, 2);
  packet.set_check_byte(rm_base::Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_gimbal_imu_frame");
  EXPECT_TRUE(
    rm_base::ProcessFactory::create(
      static_cast<uint8_t>(rm_base::RecvID::ROBOTJOINTSTATE),
      node.get()));
  EXPECT_TRUE(
    rm_base::ProcessFactory::process_packet(
      static_cast<uint8_t>(rm_base::RecvID::
      ROBOTJOINTSTATE), packet));
  rclcpp::shutdown();
}
