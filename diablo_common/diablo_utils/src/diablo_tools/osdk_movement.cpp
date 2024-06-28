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

#include "diablo_utils/diablo_tools/osdk_movement.hpp"

#include "diablo_utils/diablo_tools/osdk_header.hpp"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

using namespace std;
using namespace DIABLO::OSDK;

uint8_t Movement_Ctrl::obtain_control(uint16_t timeout_ms)
{
  if (this->in_control()) return 5;
  if (
    vehicle->telemetry->status.ctrl_mode &
    ((1 << (1 + vehicle->telemetry->id * 2)) | (1 << (5 + vehicle->telemetry->id * 2))))
    exit(0);  // control is taken by virtual RC

  Header header;
  header.data.LEN =
    sizeof(OSDK_Uart_Header_t) + sizeof(OSDK_Movement_Ctrl_Request_t) + OSDK_MISC_SIZE;
  header.data.SESSION = 2;
  header.data.ACK = 1;
  header.data.SEQ = vehicle->hal->serial_getSeq();
  header.append_crc();

  OSDK_Movement_Ctrl_Request_t req;
  req.request = 1;
  req.timeout_act = 0;
  req.timeout_ms = timeout_ms;

<<<<<<< HEAD
    uint16_t ack = -1;
    while(ack != 0x0002)
    {
        // printf("Wait ack to get AUTHORIZE\n");
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack, 
            OSDK_CONTROL_SET, OSDK_CTRL_AUTHORIZE_ID,
            &req, sizeof(OSDK_Movement_Ctrl_Request_t));
        
        if(result) return result;
        if(ack == 0x0000)
        {
            // printf("ERROR: SDK control disable on manual movement_ctrl, check your robot.\n");
            return 3;
        }
        if(ack == 0x000A)
        {
            // printf("ERROR: Cannot switch to SDK control, check your robot status.\n");
            return 3;
        }
        usleep(5000);
=======
  uint16_t ack = -1;
  while (ack != 0x0002) {
    uint8_t result = vehicle->hal->serialSend_ack(
      header.data, ack, OSDK_CONTROL_SET, OSDK_CTRL_AUTHORIZE_ID, &req,
      sizeof(OSDK_Movement_Ctrl_Request_t));

    if (result) return result;
    if (ack == 0x0000) {
      printf("ERROR: SDK control disable on manual movement_ctrl, check your robot.\n");
      return 3;
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918
    }
    if (ack == 0x000A) {
      printf("ERROR: Cannot switch to SDK control, check yout robot status.\n");
      return 3;
    }
    usleep(5000);
  }

  this->ctrl_status = CTRL_OBTAINED;
  idle_buffer = true;
  printf("SDK Handle Movement control\n");
  // abort();
  return 0;
}

uint8_t Movement_Ctrl::release_control()
{
  Header header;
  header.data.LEN =
    sizeof(OSDK_Uart_Header_t) + sizeof(OSDK_Movement_Ctrl_Request_t) + OSDK_MISC_SIZE;
  header.data.SESSION = 2;
  header.data.ACK = 1;
  header.data.SEQ = vehicle->hal->serial_getSeq();
  header.append_crc();

  OSDK_Movement_Ctrl_Request_t req;
  req.request = 0;
  req.timeout_act = 0;
  req.timeout_ms = 0;

  vehicle->hal->serialSend(
    header.data, OSDK_CONTROL_SET, OSDK_CTRL_AUTHORIZE_ID, &req,
    sizeof(OSDK_Movement_Ctrl_Request_t));

  this->ctrl_status = CTRL_RELEASED;
  this->vehicle->telemetry->connection_check();
  return 0;
}


void Movement_Ctrl::CtrlStatusMonitorHandle(const uint8_t ctrl_mode)
<<<<<<< HEAD
{ 
    if(ctrl_mode == 1)
    {
        ctrl_status = CTRL_OBTAINED;
        dropCtrlCnt = 0;
    }
    else        //if(ctrl_mode != OSDK_CTRL_MODE_MOTION)
    {
        if(dropCtrlCnt > 10)
        {
            if(ctrl_status == CTRL_OBTAINED)
                std::cout<<"Motion control authority released by vehicle"<<std::endl;
            this->SerialDisconnectHandle();
        }
        else
            dropCtrlCnt++;
=======
{
  if (ctrl_mode & (1 << (vehicle->telemetry->id * 2))) {
    ctrl_status = CTRL_OBTAINED;
    dropCtrlCnt = 0;
  } else if (ctrl_mode & (1 << (4 + vehicle->telemetry->id * 2)))  // another sdk handles control
  {
    if (ctrl_mode & 0x0F)
      ctrl_status = CTRL_IDLE;
    else
      ctrl_status = CTRL_RELEASED;
    dropCtrlCnt = 0;
  } else  // if(ctrl_mode != OSDK_CTRL_MODE_MOTION)
  {
    if (dropCtrlCnt > 10) {
      if (ctrl_status == CTRL_OBTAINED)
        std::cout << "Motion control authority released by vehicle" << std::endl;
      this->SerialDisconnectHandle();
    } else {
      dropCtrlCnt++;
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918
    }
  }
}

uint8_t Movement_Ctrl::EmergencyBrake(void)
{
  Header header;
  header.data.LEN = sizeof(OSDK_Uart_Header_t) + sizeof(OSDK_Brake_Cmd_t) + OSDK_MISC_SIZE;
  header.data.SESSION = 2;
  header.data.ACK = 1;
  header.data.SEQ = vehicle->hal->serial_getSeq();
  header.append_crc();

  OSDK_Brake_Cmd_t req = OSDK_ENABLE;

  uint16_t ack = -1;
  while (ack != OSDK_ENABLE) {
    uint8_t result = vehicle->hal->serialSend_ack(
      header.data, ack, OSDK_CONTROL_SET, OSDK_EMERGENCY_BRAKE_ID,
      reinterpret_cast<uint8_t *>(&req), sizeof(OSDK_Brake_Cmd_t));

    if (result) return result;
  }

  std::cout << "==EMERGENCY BRAKE ACTIVATED==" << std::endl;
  return 0;
}

uint8_t Movement_Ctrl::EmergencyBrakeRelease(void)
{
  Header header;
  header.data.LEN = sizeof(OSDK_Uart_Header_t) + sizeof(OSDK_Brake_Cmd_t) + OSDK_MISC_SIZE;
  header.data.SESSION = 2;
  header.data.ACK = 1;
  header.data.SEQ = vehicle->hal->serial_getSeq();
  header.append_crc();

  OSDK_Brake_Cmd_t req = OSDK_DISABLE;

  uint16_t ack = -1;
  while (ack != OSDK_DISABLE) {
    uint8_t result = vehicle->hal->serialSend_ack(
      header.data, ack, OSDK_CONTROL_SET, OSDK_EMERGENCY_BRAKE_ID,
      reinterpret_cast<uint8_t *>(&req), sizeof(OSDK_Brake_Cmd_t));

    if (result) return result;
  }

  std::cout << "==EMERGENCY BRAKE RELEASED==" << std::endl;
  return 0;
}

uint8_t Movement_Ctrl::SendMovementModeCtrlCmd()
{
  if (ctrl_status == CTRL_IDLE) {
    if (idle_buffer) {
      printf("Another SDK takes the control. Current SDK is set to idle\n");
      this->vehicle->telemetry->connection_check();
      idle_buffer = false;
    }
<<<<<<< HEAD

    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) +
        sizeof(OSDK_Movement_Ctrl_Mode_t) + OSDK_MISC_SIZE;
    header.data.SESSION = 2;
    header.data.ACK = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    uint16_t ack =-1;
    while(ack != 0x0000)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
            OSDK_CONTROL_SET, OSDK_MOVEMENT_CTRL_MODE_ID,
            &ctrl_mode_data, sizeof(OSDK_Movement_Ctrl_Mode_t));

        if(result) return result;
    }
    ctrl_mode_cmd = false;
=======
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918
    return 0;
  }

  Header header;
  header.data.LEN = sizeof(OSDK_Uart_Header_t) + sizeof(OSDK_Movement_Ctrl_Mode_t) + OSDK_MISC_SIZE;
  header.data.SESSION = 2;
  header.data.ACK = 1;
  header.data.SEQ = vehicle->hal->serial_getSeq();
  header.append_crc();

  uint16_t ack = -1;
  while (ack != 0x0000) {
    uint8_t result = vehicle->hal->serialSend_ack(
      header.data, ack, OSDK_CONTROL_SET, OSDK_MOVEMENT_CTRL_MODE_ID, &ctrl_mode_data,
      sizeof(OSDK_Movement_Ctrl_Mode_t));

    if (result) return result;
  }

  printf("==MOVEMENT CONTROL MODE SET==\n");
  ctrl_mode_cmd = false;
  return 0;
}


uint8_t Movement_Ctrl::SendMovementCtrlCmd()
{
  if (ctrl_status == CTRL_IDLE) {
    if (idle_buffer) {
      printf("Another SDK takes the control. Current SDK is set to idle\n");
      this->vehicle->telemetry->connection_check();
      idle_buffer = false;
    }
    return 0;
  }

  Header header;
  header.data.LEN = sizeof(OSDK_Uart_Header_t) + sizeof(OSDK_Movement_Ctrl_t) + OSDK_MISC_SIZE;
  header.data.SEQ = vehicle->hal->serial_getSeq();
  header.append_crc();

  return vehicle->hal->serialSend(
    header.data, OSDK_CONTROL_SET, OSDK_MOVEMENT_CTRL_ID, &ctrl_data, sizeof(OSDK_Movement_Ctrl_t));
}

uint8_t Movement_Ctrl::SendTransformUpCmd()
{
  if (ctrl_status == CTRL_IDLE) {
    if (idle_buffer) {
      printf("Another SDK takes the control. Current SDK is set to idle\n");
      this->vehicle->telemetry->connection_check();
      idle_buffer = false;
    }
<<<<<<< HEAD

    transform_data.transform_down = 0;
    transform_data.transform_up = 1;
    transform_data.jump = 0 ;
    transform_data.automation = 0 ; //Azhen
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Transform_Cmd_t) + OSDK_MISC_SIZE;
    header.data.SESSION  = 2;
    header.data.ACK = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    uint16_t ack = -1;
    while(ack != 1)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
            OSDK_CONTROL_SET, OSDK_TRANSFORM_ID,
            &transform_data, sizeof(OSDK_Transform_Cmd_t));
        
        if(result) return result;

        if(ack == 0)
        {
            // printf("==TRANSFORM UP FAIL==\n");
            return 0xFF;
        }
        usleep(10000);
    }

=======
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918
    return 0;
  }

  transform_data.transform_down = 0;
  transform_data.transform_up = 1;
  Header header;
  header.data.LEN = sizeof(OSDK_Uart_Header_t) + sizeof(OSDK_Transform_Cmd_t) + OSDK_MISC_SIZE;
  header.data.SESSION = 2;
  header.data.ACK = 1;
  header.data.SEQ = vehicle->hal->serial_getSeq();
  header.append_crc();

  uint16_t ack = -1;
  while (ack != 1) {
    uint8_t result = vehicle->hal->serialSend_ack(
      header.data, ack, OSDK_CONTROL_SET, OSDK_TRANSFORM_ID, &transform_data,
      sizeof(OSDK_Transform_Cmd_t));

    if (result) return result;

    if (ack == 0) {
      printf("==TRANSFORM UP FAIL==\n");
      return 0xFF;
    }
    usleep(10000);
  }

  return 0;
}



uint8_t Movement_Ctrl::SendJumpCmd(uint8_t Jump_mark)
{
    
    transform_data.jump = Jump_mark;
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Transform_Cmd_t) + OSDK_MISC_SIZE;
    header.data.SESSION  = 2;
    header.data.ACK = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    uint16_t ack = -1;
    while(ack != 1)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
            OSDK_CONTROL_SET, OSDK_TRANSFORM_ID,
            &transform_data, sizeof(OSDK_Transform_Cmd_t));
        
        if(result) return result;

        if(ack == 0)
        {
            return 0xFF;
        }
        usleep(10000);
    }

    return 0;
}


uint8_t Movement_Ctrl::SendDanceCmd(uint8_t dance_mark)
{
    if(dance_mark){
        transform_data.automation = 2;
    }else{
        transform_data.automation = 0;
    }
    Header header;
    header.data.LEN = sizeof(OSDK_Uart_Header_t) + 
        sizeof(OSDK_Transform_Cmd_t) + OSDK_MISC_SIZE;
    header.data.SESSION  = 2;
    header.data.ACK = 1;
    header.data.SEQ = vehicle->hal->serial_getSeq();
    header.append_crc();

    uint16_t ack = -1;
    while(ack != 1)
    {
        uint8_t result = vehicle->hal->serialSend_ack(header.data, ack,
            OSDK_CONTROL_SET, OSDK_TRANSFORM_ID,
            &transform_data, sizeof(OSDK_Transform_Cmd_t));
        
        if(result) return result;

        if(ack == 0)
        {
            return 0xFF;
        }
        usleep(10000);
    }
    
    return 0;
}


uint8_t Movement_Ctrl::SendTransformDownCmd()
{
  if (ctrl_status == CTRL_IDLE) {
    if (idle_buffer) {
      printf("Another SDK takes the control. Current SDK is set to idle\n");
      this->vehicle->telemetry->connection_check();
      idle_buffer = false;
    }
    return 0;
  }

  transform_data.transform_down = 1;
  transform_data.transform_up = 0;
  Header header;
  header.data.LEN = sizeof(OSDK_Uart_Header_t) + sizeof(OSDK_Transform_Cmd_t) + OSDK_MISC_SIZE;
  header.data.SESSION = 2;
  header.data.ACK = 1;
  header.data.SEQ = vehicle->hal->serial_getSeq();
  header.append_crc();

  uint16_t ack = -1;
  while (ack != 0) {
    uint8_t result = vehicle->hal->serialSend_ack(
      header.data, ack, OSDK_CONTROL_SET, OSDK_TRANSFORM_ID, &transform_data,
      sizeof(OSDK_Transform_Cmd_t));

    if (result) return result;

    if (ack == 1) {
      printf("==TRANSFORM DOWN FAIL==\n");
      return 0xFF;
    }
    usleep(10000);
  }

  return 0;
}
