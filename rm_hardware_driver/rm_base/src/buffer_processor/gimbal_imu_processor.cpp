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

#include <tf2_ros/transform_broadcaster.h>

#include "rm_base/buffer_processor_factory.hpp"

#include <rm_base/protocol_types.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace rm_base
{

typedef struct __packed
{
  // 姿态四元数
  float ori_x;
  float ori_y;
  float ori_z;
  float ori_w;
  // 角速度
  float angular_v_x;
  float angular_v_y;
  float angular_v_z;
  // 线速度
  float acc_x;
  float acc_y;
  float acc_z;
} __attribute__((__packed__)) imu_t;

class GimbalIMUProcessor : public ProcessInterface
{
public:
  explicit GimbalIMUProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    pub_ =
      node_->create_publisher<sensor_msgs::msg::Imu>(
      "gimbal_imu", rclcpp::QoS(
        rclcpp::KeepLast(
          1)));
    pub_tf_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto timestamp = node_->now();
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      if (!packet_recv.unload_data(data_, 2)) {
        RCLCPP_WARN(node_->get_logger(), "Unload data failed for GimbalIMU processor.");
        return false;
      }
      imu_.header.stamp = timestamp;
      imu_.header.frame_id = "imu_link";
      imu_.orientation.x = data_.ori_x;
      imu_.orientation.y = data_.ori_y;
      imu_.orientation.z = data_.ori_z;
      imu_.orientation.w = data_.ori_w;
      imu_.angular_velocity.x = data_.angular_v_x;
      imu_.angular_velocity.y = data_.angular_v_y;
      imu_.angular_velocity.z = data_.angular_v_z;
      imu_.linear_acceleration.x = data_.acc_x;
      imu_.linear_acceleration.y = data_.acc_y;
      imu_.linear_acceleration.z = data_.acc_z;
      pub_->publish(imu_);

      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = timestamp;
      transform_stamped.header.frame_id = "imu_link";
      transform_stamped.child_frame_id = "gimbal_odom";
      transform_stamped.transform.translation.x = 0.0;
      transform_stamped.transform.translation.y = 0.0;
      transform_stamped.transform.translation.z = 0.0;
      transform_stamped.transform.rotation.x = -data_.ori_x;
      transform_stamped.transform.rotation.y = -data_.ori_y;
      transform_stamped.transform.rotation.z = -data_.ori_z;
      transform_stamped.transform.rotation.w = data_.ori_w;
      pub_tf_->sendTransform(transform_stamped);
    }
    return true;
  }

private:
  imu_t data_;
  sensor_msgs::msg::Imu imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> pub_tf_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(GimbalIMUProcessor, static_cast<uint8_t>(RecvID::GIMBALIMU))

}  // namespace rm_base
