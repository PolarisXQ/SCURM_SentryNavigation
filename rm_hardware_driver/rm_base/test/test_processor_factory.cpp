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

#include <rm_base/buffer_processor_factory.hpp>

class TestProcessor : public rm_base::ProcessInterface
{
public:
  explicit TestProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
  }

  bool process_packet(const Packet & packet) override
  {
    (void) packet;
    return true;
  }
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(TestProcessor, 0xff)


TEST(TestProcessorFactory, register_class)
{
  EXPECT_TRUE(TestProcessor_registered);
}

TEST(TestProcessorFactory, create_class)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  EXPECT_TRUE(rm_base::ProcessFactory::create(0xff, node.get()));
  rclcpp::shutdown();
}

TEST(TestProcessorFactory, process_packet)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rm_base::ProcessFactory::create(0xff, node.get());
  EXPECT_TRUE(rm_base::ProcessFactory::process_packet(0xff, rmoss_base::FixedPacket16()));
  rclcpp::shutdown();
}
