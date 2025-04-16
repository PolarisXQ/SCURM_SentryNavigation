// Copyright 2022 Yunlong Feng
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

#include <rm_base/buffer_processor_factory.hpp>

#include <rm_base/protocol_types.hpp>
#include "rm_interfaces/msg/game_status.hpp"

namespace rm_base
{

typedef struct
{
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
} __attribute__((__packed__)) ext_game_status_t;

class GameStatusProcessor : public ProcessInterface
{
public:
  explicit GameStatusProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("game_status_topic", "game_status");
    RCLCPP_INFO(node_->get_logger(), "game_status_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::GameStatus>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::GameStatus::UniquePtr msg(new rm_interfaces::msg::GameStatus());
      msg->header.frame_id = "base_link";
      msg->header.stamp = node_->get_clock()->now();

      ext_game_status_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->game_type = data.game_type;
      msg->game_progress = data.game_progress;
      msg->stage_remain_time = data.stage_remain_time;
      msg->sync_time_stamp = data.SyncTimeStamp;

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for GameStatus processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::GameStatus>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(GameStatusProcessor, static_cast<uint8_t>(RecvID::GAMESTATUS))

}  // namespace rm_base
