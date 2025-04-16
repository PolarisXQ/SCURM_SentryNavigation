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

#include "rm_base/buffer_processor_factory.hpp"

#include <rm_base/protocol_types.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rm_base
{

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

class JointStateProcessor : public ProcessInterface
{
public:
  explicit JointStateProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    pub_ =
      node_->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states",
      rclcpp::QoS(rclcpp::KeepLast(1)));
    joint_state_.name.resize(6);
    joint_state_.position.resize(6);
    joint_state_.velocity.resize(6);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      joint_state_.header.stamp = node_->now();

      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      if (!packet_recv.unload_data(data_, 2)) {
        RCLCPP_WARN(node_->get_logger(), "Unload data failed for JointState processor.");
        return false;
      }
      joint_state_.name[0] = "yaw_joint";
      joint_state_.position[0] = data_.yaw_position;
      joint_state_.velocity[0] = data_.yaw_velocity;
      joint_state_.name[1] = "pitch_joint";
      joint_state_.position[1] = data_.pitch_position;
      joint_state_.velocity[1] = data_.pitch_velocity;
      joint_state_.name[2] = "wheel_left_front_joint";
      joint_state_.position[2] = data_.wheel_left_front_position;
      joint_state_.velocity[2] = data_.wheel_left_front_velocity;
      joint_state_.name[3] = "wheel_left_back_joint";
      joint_state_.position[3] = data_.wheel_left_rear_position;
      joint_state_.velocity[3] = data_.wheel_left_rear_velocity;
      joint_state_.name[4] = "wheel_right_front_joint";
      joint_state_.position[4] = data_.wheel_right_front_position;
      joint_state_.velocity[4] = data_.wheel_right_front_velocity;
      joint_state_.name[5] = "wheel_right_back_joint";
      joint_state_.position[5] = data_.wheel_right_rear_position;
      joint_state_.velocity[5] = data_.wheel_right_rear_velocity;
      pub_->publish(joint_state_);
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for JointState processor.");
      return false;
    }
  }

private:
  robot_joint_state_t data_;
  sensor_msgs::msg::JointState joint_state_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(JointStateProcessor, static_cast<uint8_t>(RecvID::ROBOTJOINTSTATE))

}  // namespace rm_base
