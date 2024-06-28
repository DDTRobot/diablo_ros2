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

#ifndef DIABLO_UTILS__DIABLO_TOOLS__OSDK_VIRTUAL_RC_HPP_
#define DIABLO_UTILS__DIABLO_TOOLS__OSDK_VIRTUAL_RC_HPP_

#include <cstdio>
#include <cstring>
#include <iostream>

#include "onboard_sdk_uart_protocol.h"

namespace DIABLO
{
namespace OSDK
{

// Forward Declaration
class HAL;
class Vehicle;

class Virtual_RC
{
public:
  explicit Virtual_RC(Vehicle * vehicle)
  : vehicle(vehicle), ctrl_status(CTRL_RELEASED), dropCtrlCnt(0), idle_buffer(false)
  {
    memset(&data, 0, sizeof(OSDK_Virtual_RC_t));
  }

  ~Virtual_RC()
  {
    if (this->release_control()) std::cerr << "Failed to release control Virtual RC" << std::endl;
  }

  enum ctrl_status_t {
    CTRL_RELEASED = 0,  // release control
    CTRL_PENDING = 1,   // waiting for authorization from controller board
    CTRL_OBTAINED = 2,  // obtain control
    CTRL_IDLE = 3       // another SDK controller obtain the control
  };

  /**
     * @brief   reset virtual RC data structure
     */
  void reset_data(void)
  {
    uint16_t * chn = reinterpret_cast<uint16_t *>(&data);
    for (uint8_t i = 0; i < 12; i++) chn[i] = 1000;
    uint8_t * p = reinterpret_cast<uint8_t *>(&data) + 24;
    *p = 0;
  }

  /**
     * @brief     obtain control authority of the vehicle using virtual RC cmd
     * @param[in] timeout_ms : automatically release control authority after not receiving valid message for some milliseconds
     * @note      this function will automatically transmit two packets of "obtain control" frame. If control authorization is given to movement ctrl. it will exit the program
     */
  uint8_t obtain_control(uint16_t timeout_ms = 500);

  /**
     * @brief   release control authority of the vehicle
     */
  uint8_t release_control();

  /**
     * @brief check control authority status of virtual RC
     * @return true if control authority has been acquired
     */
  bool in_control(void) { return (ctrl_status == CTRL_OBTAINED || ctrl_status == CTRL_IDLE); }

  /**
     * @brief send virtual RC command to serial port
     * @note  this function process serial transmission only, in non-blocking mode
     */
  uint8_t SendVirtualRCCmd();

  /**
     * @brief handle serial disconnection event
     * @note  NON-API FUNCTION
     */
  void SerialDisconnectHandle()
  {
    this->ctrl_status = CTRL_RELEASED;
    this->reset_data();
  }

  /**
     * @brief Monitor the control mode and handle disconnecting issue
     * @note NON-API FUNCTION
     * @param[in] ctrl_mode : control mode of the vehicle
     */
  void CtrlStatusMonitorHandle(const uint8_t ctrl_mode);

public:
  OSDK_Virtual_RC_t data; /**< Virtual rc command data, edit before transmission */

private:
  Vehicle * vehicle;
  ctrl_status_t ctrl_status;  // control mode of current SDK controller
  uint8_t dropCtrlCnt;

  bool idle_buffer;  // current SDK is idle or not
};

}  // namespace OSDK
}  // namespace DIABLO

#endif  // DIABLO_UTILS__DIABLO_TOOLS__OSDK_VIRTUAL_RC_HPP_
