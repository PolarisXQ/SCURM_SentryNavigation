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
#include "rm_interfaces/msg/robot_buff.hpp"

namespace rm_base
{

typedef struct __packed
{
  uint8_t recovery_buff;
  uint8_t cooling_buff;
  uint8_t defence_buff;
  uint8_t vulnerability_buff;
  uint16_t attack_buff;
} __attribute__((__packed__)) ext_buff_t;

class RobotBuffProcessor : public ProcessInterface
{
public:
  explicit RobotBuffProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("robor_buff_topic", "robor_buff");
    RCLCPP_INFO(node_->get_logger(), "robor_buff_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::RobotBuff>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::RobotBuff::UniquePtr msg(new rm_interfaces::msg::RobotBuff());

      ext_buff_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->recovery_buff = data.recovery_buff;
      msg->cooling_buff = data.cooling_buff;
      msg->defence_buff = data.defence_buff;
      msg->vulnerability_buff = data.vulnerability_buff;
      msg->attack_buff = data.attack_buff;
      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for RobotBuff processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::RobotBuff>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(RobotBuffProcessor, static_cast<uint8_t>(RecvID::ROBOTBUFF))

}  // namespace rm_base
