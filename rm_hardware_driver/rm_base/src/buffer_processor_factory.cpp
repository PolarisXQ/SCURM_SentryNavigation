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

#include <rm_base/buffer_processor_factory.hpp>

namespace rm_base
{

bool ProcessFactory::create(const uint8_t & key, rclcpp::Node * node)
{
  auto processor = get_processor_map().find(key);
  auto creator = get_creator_map().find(key);
  if (processor == get_processor_map().end() && creator != get_creator_map().end()) {
    get_processor_map()[key] = creator->second(node);
    return true;
  }
  return false;
}

bool ProcessFactory::process_packet(const uint8_t & key, const ProcessInterface::Packet & packet)
{
  if (auto it = get_processor_map().find(key); it != get_processor_map().end()) {
    return it->second->process_packet(packet);
  }
  return false;
}

std::vector<ProcessInterface::SharedPtr> ProcessFactory::get_processors()
{
  std::vector<ProcessInterface::SharedPtr> processors;
  for (auto & processor : get_processor_map()) {
    processors.push_back(processor.second);
  }
  return processors;
}

std::vector<uint8_t> ProcessFactory::get_creator_keys()
{
  std::vector<uint8_t> keys;
  for (auto & processor : get_creator_map()) {
    keys.push_back(processor.first);
  }
  return keys;
}

std::unordered_map<uint8_t, ProcessFactory::CreateFunction> & ProcessFactory::get_creator_map()
{
  static std::unordered_map<uint8_t, CreateFunction> creator_map;
  return creator_map;
}

std::unordered_map<uint8_t, ProcessInterface::SharedPtr> & ProcessFactory::get_processor_map()
{
  static std::unordered_map<uint8_t, ProcessInterface::SharedPtr> processor_map;
  return processor_map;
}

}  // namespace rm_base
