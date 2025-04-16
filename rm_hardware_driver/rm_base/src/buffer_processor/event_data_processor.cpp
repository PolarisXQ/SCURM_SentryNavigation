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
#include "rm_interfaces/msg/event_data.hpp"

namespace rm_base
{

  typedef struct
  {
    uint32_t event_data;
  } __attribute__((__packed__)) ext_event_data_t;

  class EventDataProcessor : public ProcessInterface
  {
  public:
    explicit EventDataProcessor(rclcpp::Node *node)
        : ProcessInterface(node)
    {
      auto topic_name = node_->declare_parameter("event_data_topic", "event_data");
      RCLCPP_INFO(node_->get_logger(), "event_data_topic: %s", topic_name.c_str());
      pub_ = node_->create_publisher<rm_interfaces::msg::EventData>(topic_name, 10);
    }

    bool process_packet(const Packet &packet)
    {
      if (std::holds_alternative<rmoss_base::FixedPacket64>(packet))
      {
        auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
        rm_interfaces::msg::EventData::UniquePtr msg(new rm_interfaces::msg::EventData());

        ext_event_data_t data;
        if (!packet_recv.unload_data(data, 2))
        {
          return false;
        }
        msg->supply_1_occupation = get_event_data(data, 0);
        msg->supply_2_occupation = get_event_data(data, 1);
        msg->supply_3_occupation = get_event_data(data, 2);
        msg->rune_hit_occupation = get_event_data(data, 3);
        msg->rune_small_enable = get_event_data(data, 4);
        msg->rune_big_enable = get_event_data(data, 5);
        msg->highland_1_occupation = get_event_data(data, 6) * 1 
                                    + get_event_data(data, 7) * 2;
        msg->highland_2_occupation = get_event_data(data, 8) * 1 
                                    + get_event_data(data, 9) * 2;
        msg->highland_3_occupation = get_event_data(data, 10) * 1 
                                    + get_event_data(data, 11) * 2;
        msg->base_shield_remain = get_event_data(data, 12) * 1 
                                    + get_event_data(data, 13) * 2 
                                    + get_event_data(data, 14) * 4 
                                    + get_event_data(data, 15) * 8 
                                    + get_event_data(data, 16) * 16 
                                    + get_event_data(data, 17) * 32 
                                    + get_event_data(data, 18) * 64;
        msg->dart_last_hit_time = get_event_data(data, 19) * 1 
                                    + get_event_data(data, 20) * 2 
                                    + get_event_data(data, 21) * 4 
                                    + get_event_data(data, 22) * 8 
                                    + get_event_data(data, 23) * 16 
                                    + get_event_data(data, 24) * 32 
                                    + get_event_data(data, 25) * 64 
                                    + get_event_data(data, 26) * 128 
                                    + get_event_data(data, 27) * 256;
        msg->dart_last_hit_object = get_event_data(data, 28) * 1 
                                    + get_event_data(data, 29) * 2;
        msg->center_buff_occupation = get_event_data(data, 30) * 1 
                                    + get_event_data(data, 31) * 2;
        pub_->publish(std::move(msg));
        return true;
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for EventData processor.");
        return false;
      }
    }

  private:
    bool get_event_data(ext_event_data_t data, int bit)
    {
      return (data.event_data >> bit) & 0x00000001;
    }

    rclcpp::Publisher<rm_interfaces::msg::EventData>::SharedPtr pub_;
  };

#include <rm_base/register_macro.hpp>

  REGISTER_PROCESSOR_CLASS(EventDataProcessor, static_cast<uint8_t>(RecvID::EVENTDATA))

} // namespace rm_base
