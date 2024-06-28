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

#include "osdk_hal.hpp"
#include "osdk_movement.hpp"
#include "osdk_telemetry.hpp"
<<<<<<< HEAD
=======
#include "osdk_vehicle.hpp"
#include "osdk_virtual_rc.hpp"
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918

namespace DIABLO
{
namespace OSDK
{

class Vehicle
{
<<<<<<< HEAD
public: 
    Vehicle(HAL* hal): hal(hal),
    movement_ctrl(NULL), 
    telemetry(NULL)
    {}

    ~Vehicle()
    {
        if(movement_ctrl) delete movement_ctrl;
        if(telemetry)     delete telemetry;
    }
=======
public:
  explicit Vehicle(HAL * hal) : hal(hal), movement_ctrl(NULL), virtual_rc(NULL), telemetry(NULL) {}

  ~Vehicle()
  {
    if (movement_ctrl) delete movement_ctrl;
    if (virtual_rc) delete virtual_rc;
    if (telemetry) delete telemetry;
  }
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918

  /**
     * @brief   Initialize SDK port to vehicle
     */
  uint8_t init(void);

public:
<<<<<<< HEAD
    HAL*               hal;
    
public:
    Movement_Ctrl*  movement_ctrl;
    Telemetry*      telemetry;
=======
  HAL * hal;
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918

public:
  Movement_Ctrl * movement_ctrl;
  Virtual_RC * virtual_rc;
  Telemetry * telemetry;
};

}  // namespace OSDK
}  // namespace DIABLO
