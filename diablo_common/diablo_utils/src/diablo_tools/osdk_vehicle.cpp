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

#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

using namespace DIABLO::OSDK;

/**
 * @brief   Initialize SDK port to vehicle
 */
uint8_t Vehicle::init(void)
{
  movement_ctrl = new Movement_Ctrl(this);
  if (!movement_ctrl) {
    std::cerr << "Failed to allocate memory for Movement_Ctrl!\n" << std::endl;
    return 1;
  }

  virtual_rc = new Virtual_RC(this);
  if (!virtual_rc) {
    std::cerr << "Failed to allocate memory for Virtual RC!\n" << std::endl;
    return 1;
  }

  telemetry = new Telemetry(this);
  if (!telemetry) {
    std::cerr << "Failed to allocate memory for Telemetry!\n" << std::endl;
    return 1;
  }

  return 0;
}
