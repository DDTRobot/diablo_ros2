// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

#pragma once

#include <cstdio>
#include <cstring>
#include <iostream>

#include "onboard_sdk_uart_protocol.h"
#include "osdk_crc.hpp"

namespace DIABLO
{
namespace OSDK
{
struct Header
{
public:
  Header()
  {
    memset(&data, 0, sizeof(OSDK_Uart_Header_t));
    data.SOF = OSDK_HEADER;
  }

  /**
     * @brief append crc16 for packet head
     * @note NON-API FUNCTION
     */
  void append_crc(void)
  {
    data.CRC16 = DIABLO::Utility::update_crc16(&this->data, sizeof(OSDK_Uart_Header_t) - 2);
  }

  /**
     * @brief verify receiving packet's head crc16
     * 
     * @return true head data is correct
     * @return false head data has been disturbed
     */
  bool verify(void)
  {
    return this->data.SOF == OSDK_HEADER &&
           DIABLO::Utility::verify_crc16(&this->data, sizeof(OSDK_Uart_Header_t));
    //  return this->data.SOF == OSDK_HEADER;
  }

public:
  OSDK_Uart_Header_t data;
};
}  // namespace OSDK
}  // namespace DIABLO
