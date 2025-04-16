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

#ifndef RM_BASE__BUFFER_PROCESSOR_FACTORY_HPP_
#define RM_BASE__BUFFER_PROCESSOR_FACTORY_HPP_

#include <string>
#include <functional>
#include <unordered_map>
#include <variant>
#include <memory>
#include <vector>

#include <rm_base/processor_interface.hpp>

namespace rm_base
{

class ProcessFactory
{
  /**
   * @brief 串口处理工厂类
   * 根据不同的key(消息包id)来生成对应的处理对象
   */

public:
  using CreateFunction = std::function<ProcessInterface::SharedPtr(rclcpp::Node * node)>;

  /**
   * @brief 注册处理类的派生类构造函数
   *
   * @tparam T
   * @param key
   * @return true
   * @return false
   */
  template<typename T>
  static bool register_class(const uint8_t & key)
  {
    if (auto it = get_creator_map().find(key); it == get_creator_map().end()) {
      get_creator_map()[key] = [](rclcpp::Node * node) {return std::make_shared<T>(node);};
      return true;
    }
    return false;
  }

  /**
   * @brief 根据 Key 创建新的帧处理对象，并添加进处理
   *
   * @param key 帧标志id
   * @param node 处理帧的ROS节点
   * @return true
   * @return false
   */
  static bool create(const uint8_t & key, rclcpp::Node * node);

  /**
   * @brief 解析具体消息帧
   *
   * @param key 帧标志id
   * @param packet 完整的帧内容
   * @return true
   * @return false
   */
  static bool process_packet(const uint8_t & key, const ProcessInterface::Packet & packet);

  /**
   * @brief Get the processors object
   *
   * @return std::vector<ProcessInterface::SharedPtr>
   */
  static std::vector<ProcessInterface::SharedPtr> get_processors();

  /**
   * @brief Get the keys object
   *
   * @return std::vector<std::uint8_t>
   */
  static std::vector<std::uint8_t> get_creator_keys();

private:
  static std::unordered_map<uint8_t, CreateFunction> & get_creator_map();

  static std::unordered_map<uint8_t, ProcessInterface::SharedPtr> & get_processor_map();
};
}  // namespace rm_base

#endif  // RM_BASE__BUFFER_PROCESSOR_FACTORY_HPP_
