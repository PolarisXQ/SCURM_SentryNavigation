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

#include "rm_base/robot_base_node.hpp"

#include "rmoss_base/uart_transporter.hpp"

#include "rm_base/buffer_processor_factory.hpp"
#include "rm_base/crc.hpp"
#include "rm_base/protocol_types.hpp"

namespace rm_base
{

typedef struct
{
  uint8_t type;
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
} __attribute__((__packed__)) chassis_cmd_t;

typedef struct
{
  uint8_t yaw_type;
  uint8_t pitch_type;
  float position_yaw;
  float position_pitch;
  float velocity_yaw;
  float velocity_pitch;
} __attribute__((__packed__)) gimbal_cmd_t;

typedef struct
{
  uint8_t type;
  uint8_t projectile_num;
} __attribute__((__packed__)) shoot_cmd_t;

typedef struct
{
  bool resurrection_en;
  bool buy_resurrection_en;
  uint16_t buy_projectile_allowance;
  uint16_t buy_projectile_times;
  uint16_t buy_hp_times;
} __attribute__((__packed__)) sentry_cmd_t;

RobotBaseNode::RobotBaseNode(const rclcpp::NodeOptions & options)
: Node("robot_base_node", options)
{
  // declare parameters
  auto port_name = this->declare_parameter("port", "/dev/ttyUSB0");
  int baud_rate = this->declare_parameter("baud_rate", 115200);
  bool enable_realtime_send = this->declare_parameter("enable_realtime_send", false);
  auto gimbal_cmd_topic_name = this->declare_parameter("gimbal_cmd_topic", "gimbal_cmd");
  auto chassis_cmd_topic_name = this->declare_parameter("chassis_cmd_topic_name", "chassis_cmd");
  auto sentry_cmd_topic_name = this->declare_parameter("sentry_cmd_topic_name", "sentry_cmd");
  RCLCPP_INFO(this->get_logger(), "Serial port: %s baud_rate: %d", port_name.c_str(), baud_rate);
  RCLCPP_INFO(this->get_logger(), "enable async send: %d", enable_realtime_send);

  RCLCPP_INFO(this->get_logger(), "Initialize serial port.");
  auto transporter = std::make_shared<rmoss_base::UartTransporter>(port_name, baud_rate, 0, 8);
  if (!transporter->open()) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open serial port. Error: %s",
      transporter->error_message().c_str());
    return;
  }
  packet_tool_ = std::make_shared<rmoss_base::FixedPacketTool<64>>(transporter);
  packet_tool_->enable_realtime_send(enable_realtime_send);

  RCLCPP_INFO(this->get_logger(), "Initialize data processors.");
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::GAMESTATUS), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::GAMERESULT), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::ROBOTHP), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::EVENTDATA), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::ROBOTSTATUS), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::POWERHEATDATA), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::ROBOTPOS), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::ROBOTBUFF), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::HURTDATA), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::SHOOTDATA), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::SENTRYCMD), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::ROBOTINTERACT), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::ROBOTJOINTSTATE), this);
  // ProcessFactory::create(static_cast<uint8_t>(RecvID::GIMBALIMU), this);

  RCLCPP_INFO(this->get_logger(), "Initialize subscriptions.");
  chassis_cmd_sub_ = this->create_subscription<rm_interfaces::msg::ChassisCmd>(
    chassis_cmd_topic_name, 10,
    std::bind(&RobotBaseNode::chassis_cmd_cb, this, std::placeholders::_1));
  gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
    gimbal_cmd_topic_name, 10,
    std::bind(&RobotBaseNode::gimbal_cmd_cb, this, std::placeholders::_1));
  sentry_cmd_sub_ = this->create_subscription<rm_interfaces::msg::SentryCmd>(
    sentry_cmd_topic_name, 10,
    std::bind(&RobotBaseNode::sentry_cmd_cb, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "chassis_cmd topic: %s", chassis_cmd_sub_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "gimbal_cmd topic: %s", gimbal_cmd_sub_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "sentry_cmd topic: %s", sentry_cmd_sub_->get_topic_name());

  // task thread
  listen_thread_ = std::make_unique<std::thread>(&RobotBaseNode::listen_loop, this);
}

void RobotBaseNode::listen_loop()
{
  rmoss_base::FixedPacket<64> packet; //TODO
  while (rclcpp::ok()) {
    if (packet_tool_->recv_packet(packet)) {
      if (!verify_checksum(packet)) {
        RCLCPP_WARN(this->get_logger(), "CRC8 verify failed.");
        continue;
      }

      uint8_t cmd_id;
      packet.unload_data(cmd_id, 1);
      if (!ProcessFactory::process_packet(cmd_id, packet)) {
        RCLCPP_WARN(this->get_logger(), "No such frame cmd_key: 0x%x", cmd_id);
      }
    }
  }
}

bool RobotBaseNode::verify_checksum(const rmoss_base::FixedPacket<64> & packet)
{
  auto expected = *(packet.buffer() + 62);
  return Get_CRC8_Check_Sum(packet.buffer() + 1, 61, CRC8_INIT) == expected;
}

bool RobotBaseNode::checksum_send(rmoss_base::FixedPacket<64> & packet)
{
  // 数据段长度 61 字节
  packet.set_check_byte(Get_CRC8_Check_Sum(packet.buffer() + 1, 61, CRC8_INIT));
  return packet_tool_->send_packet(packet);
}

void RobotBaseNode::chassis_cmd_cb(const rm_interfaces::msg::ChassisCmd::SharedPtr msg)
{
  rmoss_base::FixedPacket<64> packet;
  chassis_cmd_t data;
  data.type = msg->type;
  data.linear_x = msg->twist.linear.x;
  data.linear_y = msg->twist.linear.y;
  data.linear_z = msg->twist.linear.z;
  data.angular_x = msg->twist.angular.x;
  data.angular_y = msg->twist.angular.y;
  data.angular_z = msg->twist.angular.z;
  packet.load_data(SendID::CHASSISCMD, 1);
  packet.load_data(data, 2);
  if (!this->checksum_send(packet)) {
    RCLCPP_WARN(this->get_logger(), "Send ChassisCmd failed.");
  }
}

void RobotBaseNode::gimbal_cmd_cb(const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  rmoss_base::FixedPacket<64> packet;
  gimbal_cmd_t data;
  data.yaw_type = msg->yaw_type;
  data.pitch_type = msg->pitch_type;
  data.position_yaw = msg->position.yaw;
  data.position_pitch = msg->position.pitch;
  data.velocity_yaw = msg->velocity.yaw;
  data.velocity_pitch = msg->velocity.pitch;
  packet.load_data(SendID::GIMBALCMD, 1);
  packet.load_data(data, 2);
  if (!this->checksum_send(packet)) {
    RCLCPP_WARN(this->get_logger(), "Send GimbalCmd failed.");
  }
}

void RobotBaseNode::shoot_cmd_cb(const rm_interfaces::msg::ShootCmd::SharedPtr msg)
{
  rmoss_base::FixedPacket<64> packet;
  shoot_cmd_t data;
  data.type = msg->type;
  data.projectile_num = msg->projectile_num;
  packet.load_data(SendID::SHOOTCMD, 1);
  packet.load_data(data, 2);
  if (!this->checksum_send(packet)) {
    RCLCPP_WARN(this->get_logger(), "Send ShootCmd failed.");
  }
}

void RobotBaseNode::sentry_cmd_cb(const rm_interfaces::msg::SentryCmd::SharedPtr msg)
{
  rmoss_base::FixedPacket<64> packet;
  sentry_cmd_t data;
  data.resurrection_en = msg->resurrection_en;
  data.buy_resurrection_en = msg->buy_resurrection_en;
  data.buy_projectile_allowance = msg->buy_projectile_allowance;
  data.buy_projectile_times = msg->buy_projectile_times;
  data.buy_hp_times = msg->buy_hp_times;
  packet.load_data(SendID::SENTRYCMD, 1);
  packet.load_data(data, 2);
  if (!this->checksum_send(packet)) {
    RCLCPP_WARN(this->get_logger(), "Send SentryCmd failed.");
  }
}
}  // namespace rm_base

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_base::RobotBaseNode);
