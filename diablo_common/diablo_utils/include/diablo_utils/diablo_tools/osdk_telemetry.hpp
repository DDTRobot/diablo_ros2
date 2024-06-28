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
// #define BOOST_MPL_CFG_ASSERT_BROKEN_POINTER_TO_POINTER_TO_MEMBER

#include <math.h>
#include <stdio.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "onboard_sdk_uart_protocol.h"
#include "osdk_ddj_m15.hpp"

namespace DIABLO
{
namespace OSDK
{

typedef enum {
  TOPIC_STATUS = 0,
  TOPIC_QUATERNION = 1,
  TOPIC_ACCL = 2,
  TOPIC_GYRO = 3,
  TOPIC_RC = 4,
  TOPIC_POWER = 5,
  TOPIC_MOTOR = 6
} OSDK_Topic_t;

// Forward Declaration
class HAL;
class Vehicle;

class Telemetry
{
public:
  explicit Telemetry(Vehicle * vehicle);

  ~Telemetry()
  {
    if (thd) {
      pthread_cancel(thd->native_handle());
    }
    if (cnt_thd) {
      pthread_cancel(cnt_thd->native_handle());
    }
  }

  /**
     * @brief activate telemetry serial channel
     * @note  no need to activate channel on raspberry pi
     *        activate is needed when using the custom serial port on diablo robot
     */
  uint8_t activate(void);

  /**
     * @brief robot calibration
     * @param  calibration_cmd  calibration command mode (CTRL_MODE_CALIBRATION_1 or CTRL_MODE_CALIBRATION_2)
     * @return 0: Calibration success \n 
     *         1: Low Battery \n 
     *         2: Wrong mode (robot needs to stand) \n
     */
  uint8_t Calibration(OSDK_Offline_Calibration_Cmd_t calibration_cmd);

  /** 
     * @brief configure topic update frequency and callback function
     * @param[in] topic     topic identifier enum to be configured
     * @param[in] freq      frequency enum to set
     * @param[in] callBack  a user callback function to be called upon topic update
     * @return 0: if config success
     *         1: or 2 if configuration is invalid
     */
  uint8_t configTopic(const OSDK_Topic_t topic, const OSDK_Push_Data_Freq_Select_t freq);

  /** 
     * @brief update configuration of telemetry setting. If the total bitrate exceeds the bandwidth, it will drop the changes and reset to previous configration
     * @param[in] save whether the configuration is to be saved onboard
     * @return 0: if config update success
     *         1: if error occurs
     */
  uint8_t configUpdate(const bool save = false);

  /**
    * @brief     check connection status when contorller id idle
    */
  uint8_t connection_check();

  /**
     * @brief       start log on a specific topic
     * @param[in]   topic choose topic to start log 
     */
  void enableLog(const OSDK_Topic_t topic) { log_flag |= 1 << topic; }

  /**
     * @brief calculte the total bitrate of current configuration
     * @return total bitrate
     */
  uint32_t calculatebitrate(void);

  /**
     * @brief For bridge function only: Set flag for new received data.
     * @note NON-API FUNCTION 
     */
  void setNewcomeFlag(uint8_t datacome)
  {
    std::unique_lock<std::mutex> lock(newcome_mtx);
    newcome |= datacome;
  }

  /**
     * @brief For bridge function only: Reset the flag after bridge sends the received data.
     * @note NON-API FUNCTION 
     */
  void eraseNewcomeFlag(uint8_t datasend)
  {
    std::unique_lock<std::mutex> lock(newcome_mtx);
    newcome &= datasend;
  }

  /**
     * @brief Set the Max Speed of the robot
     * @note NON-API FUNCTION. DO NOT MODIFY THIS FUNCTION
     */
  uint16_t getMaxFreq(void)
  {
    uint16_t max = 0;
    for (int i = 0; i < 7; i++) {
      if ((bit_r[i] & 0x0000FFFF) > max) max = bit_r[i] & 0x0000FFFF;
    }
    if (max == 0) max = 1;

    return max;
  }

  void setMaxSpeed(float max_speed = 1.8);

public:
  /**
     * @brief handle logging on telemetry status
     * @note  NON-API FUNCTION 
     */
  void statusLog(const OSDK_Push_Data_Status_t & status);

  /**
     * @brief handle logging on telemetry power
     * @note  NON-API FUNCTION 
     */
  void powerLog(const OSDK_Push_Data_Power_t & power);

  /**
     * @brief handle logging on telemetry quaternion
     * @note  NON-API FUNCTION
     */
  void quaternionLog(const OSDK_Push_Data_Quaternion_t & quaternion);

  /**
     * @brief handle logging on telemetry accleration
     * @note NON-API FUNCTION
     */
  void acclLog(const OSDK_Push_Data_XYZ_t & accl);

  /**
     * @brief handle logging on telemetry GYRO data
     * @note NON-API FUNCTION 
     */
  void gyroLog(const OSDK_Push_Data_XYZ_t & gyro);

  /**
     * @brief handle logging on telemetry virtual remote controller
     * @note NON-API FUNCTION 
     */
  void rcLog(const OSDK_Push_Data_RC_t & rc);

  /**
     * @brief handle logging on telemetry motor data
     * @note NON-API FUNCTION 
     */
  void motorLog(const Leg_Motors & motors);

public:
  OSDK_Push_Data_Status_t status;          // 10 uint8
  uint32_t timestamp;                      // uint32_t
  OSDK_Push_Data_Quaternion_t quaternion;  // 4 float
  OSDK_Push_Data_XYZ_t accl;               // 3 flaot
  OSDK_Push_Data_XYZ_t gyro;               // 3 flaot
  OSDK_Push_Data_RC_t rc;                  // 69 uint8
  OSDK_Push_Data_Power_t power;            // 3float 1uint8
  Leg_Motors motors;                       // 10 uint8 * 6

  uint8_t newcome;
  std::mutex newcome_mtx;

  float left_leg_length;
  float right_leg_length;
  uint8_t id;  // uart id of current SDK

private:
  uint8_t frequency_flag[7];

private:
  Vehicle * vehicle;
  std::thread * thd;
  std::thread * cnt_thd;

  bool log_start;
  uint32_t log_flag;
  uint32_t bit_r[7];

  /** 
     * @brief process serial receive of push data packet
     * @note  NON-API FUNCTION
     */
  void SerialHandle(void);

  /** 
     * @brief process serial disconnection event of push data packet
     * @note  NON-API FUNCTION
     */
  void SerialDisconnectHandle(void);

  void SDKConnectMonitor(void);
  /**
     * @brief log time information
     * @note  NON-API FUNCTION
     */
  void timeLog();
};
}  // namespace OSDK
}  // namespace DIABLO
