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

#ifndef RM_BASE__REGISTER_MACRO_HPP_
#define RM_BASE__REGISTER_MACRO_HPP_

#include <rm_base/buffer_processor_factory.hpp>

#define REGISTER_PROCESSOR_CLASS(class_name, key) \
  static bool class_name ## _registered = \
    rm_base::ProcessFactory::register_class<class_name>(key);

#endif  // RM_BASE__REGISTER_MACRO_HPP_
