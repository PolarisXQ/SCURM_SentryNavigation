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

#ifndef RM_BASE__PROTOCOL_TYPES_HPP_
#define RM_BASE__PROTOCOL_TYPES_HPP_

#include <cstdint>

namespace rm_base
{

enum class RecvID : uint8_t
{
  GAMESTATUS = 0x01,
  GAMERESULT = 0x02,
  ROBOTHP = 0x03,
  EVENTDATA = 0x04,
  ROBOTSTATUS = 0x05,
  POWERHEATDATA = 0x06,
  ROBOTPOS = 0x07,
  ROBOTBUFF = 0x08,
  HURTDATA = 0x09,
  SHOOTDATA = 0x10,
  SENTRYCMD = 0x11,
  ROBOTINTERACT = 0x12,
  ROBOTJOINTSTATE = 0x13,
  GIMBALIMU = 0x14
};

enum class SendID : uint8_t
{
  CHASSISCMD = 0x01,
  GIMBALCMD = 0x02,
  SHOOTCMD = 0x03,
  SENTRYCMD = 0x04
};

}  // namespace rm_base

#endif  // RM_BASE__PROTOCOL_TYPES_HPP_
