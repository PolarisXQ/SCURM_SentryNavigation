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
#include "rm_interfaces/msg/power_heat_data.hpp"

namespace rm_base
{

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

class PowerHeatDataProcessor : public ProcessInterface
{
public:
  explicit PowerHeatDataProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("power_heat_data_topic", "power_heat_data");
    RCLCPP_INFO(node_->get_logger(), "power_heat_data_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::PowerHeatData>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::PowerHeatData::UniquePtr msg(new rm_interfaces::msg::PowerHeatData());

      ext_power_heat_data_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->chassis_volt = data.chassis_volt;
      msg->chassis_current = data.chassis_current;
      msg->chassis_power = data.chassis_power;
      msg->buffer_energy = data.buffer_energy;
      msg->shooter_id1_17mm_cooling_heat = data.shooter_id1_17mm_cooling_heat;
      msg->shooter_id2_17mm_cooling_heat = data.shooter_id2_17mm_cooling_heat;
      msg->shooter_id1_42mm_cooling_heat = data.shooter_id1_42mm_cooling_heat;

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for PowerHeatData processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::PowerHeatData>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(PowerHeatDataProcessor, static_cast<uint8_t>(RecvID::POWERHEATDATA))

}  // namespace rm_base
