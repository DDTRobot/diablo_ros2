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

#include <iostream>

namespace DIABLO
{
namespace Utility
{

/**
 * @brief verify CRC16 value of a received data packet
 * @return true if CRC value is correct
 * @note NON-API FUNCTION
 */
bool verify_crc16(const void * data, size_t len);

/**
 * @brief calculate CRC16 value of a data packet
 * @param[in] len: shall equal to sizeof(packet) - 2 
 * @note NON-API FUNCTION
 */
uint16_t update_crc16(const void * data, size_t len);

}  // namespace Utility
}  // namespace DIABLO
