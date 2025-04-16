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
#include "rm_interfaces/msg/robot_interactive_data.hpp"
//TODO: RMUC
namespace rm_base
{

typedef struct __packed
{
  uint16_t data_cmd_id;
  uint16_t send_id;
  uint16_t recv_id;
  uint8_t data[113];  // 128字节帧限制最大数据量 113 字节
} __attribute__((__packed__)) robot_interact_t;

class RobotInteractiveDataProcessor : public ProcessInterface
{
public:
  explicit RobotInteractiveDataProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter(
      "robot_interactive_data_topic",
      "robot_interactive_data");
    RCLCPP_INFO(node_->get_logger(), "robot_interactive_data_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::RobotInteractiveData>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::RobotInteractiveData::UniquePtr msg(new rm_interfaces::msg::
        RobotInteractiveData());

      robot_interact_t referee_interact;

      if (!packet_recv.unload_data(referee_interact, 2)) {
        return false;
      }
      msg->data_cmd_id = referee_interact.data_cmd_id;
      msg->sender_id = referee_interact.send_id;
      msg->receiver_id = referee_interact.recv_id;
      for (int i = 0; i < 49; i++) {
        msg->data[i] = referee_interact.data[i];
      }

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "Invalid length of data frame for RobotInteractiveData processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::RobotInteractiveData>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(
  RobotInteractiveDataProcessor,
  static_cast<uint8_t>(RecvID::ROBOTINTERACT))

}  // namespace rm_base
