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

#ifndef RM_BASE__ROBOT_BASE_NODE_HPP_
#define RM_BASE__ROBOT_BASE_NODE_HPP_

#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rmoss_base/transporter_interface.hpp"
#include "rmoss_base/fixed_packet_tool.hpp"

#include "rm_interfaces/msg/chassis_cmd.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/shoot_cmd.hpp"
#include "rm_interfaces/msg/sentry_cmd.hpp"

namespace rm_base
{

class RobotBaseNode : public rclcpp::Node
{
public:
  explicit RobotBaseNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions().use_intra_process_comms(true));

  /**
   * @brief 串口监听线程函数
   *
   */
  void listen_loop();

private:
  void chassis_cmd_cb(const rm_interfaces::msg::ChassisCmd::SharedPtr msg);
  void gimbal_cmd_cb(const rm_interfaces::msg::GimbalCmd::SharedPtr msg);
  void shoot_cmd_cb(const rm_interfaces::msg::ShootCmd::SharedPtr msg);
  void sentry_cmd_cb(const rm_interfaces::msg::SentryCmd::SharedPtr msg);

  bool checksum_send(rmoss_base::FixedPacket<64> & packet);
  bool verify_checksum(const rmoss_base::FixedPacket<64> & packet);

private:
  // 接收线程
  std::unique_ptr<std::thread> listen_thread_;

  // 通讯工具对象
  rmoss_base::TransporterInterface::SharedPtr transporter_;
  rmoss_base::FixedPacketTool<64>::SharedPtr packet_tool_;

  // ros
  rclcpp::Subscription<rm_interfaces::msg::ChassisCmd>::SharedPtr chassis_cmd_sub_;
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<rm_interfaces::msg::ShootCmd>::SharedPtr shoot_cmd_sub_;
  rclcpp::Subscription<rm_interfaces::msg::SentryCmd>::SharedPtr sentry_cmd_sub_;
};

}  // namespace rm_base

#endif  // RM_BASE__ROBOT_BASE_NODE_HPP_
