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

#include <cstring>

#include "diablo_utils/diablo_tools/osdk_telemetry.hpp"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

using namespace std;
using namespace DIABLO::OSDK;

void Telemetry::timeLog()
{
<<<<<<< HEAD
    if(log_start) return;  //will only record timestamp once per log
    printf("\n==Timestamp:%d\n",this->timestamp);
    log_start = true;
=======
  if (log_start) return;  // will only record timestamp once per log

  // cout<<"\n==Timestamp:"<<this->timestamp<<", Delay:"<<
  //     (vehicle->hal->getTimeStamp() - this->timestamp)*1e3<<"ms==\n";
  printf("\n==Timestamp:%d\n", this->timestamp);
  log_start = true;
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918
}

void Telemetry::statusLog(const OSDK_Push_Data_Status_t & status)
{
  this->timeLog();
  string ctrlMode, robotMode;

  switch (status.ctrl_mode) {
    case OSDK_CTRL_MODE_RC:
      ctrlMode = "RC";
      break;
    case OSDK_CTRL_MODE_MOTION:
      ctrlMode = "SDK Motion";
      break;
    case OSDK_CTRL_MODE_VIRTUAL_RC:
      ctrlMode = "SDK Virtual RC";
      break;
    default:
      ctrlMode = "RC";
      break;
  }

  switch (status.robot_mode) {
    case OSDK_ROBOT_STATE_DISCONNECT:
      robotMode = "Disconnect";
      break;
    case OSDK_ROBOT_STATE_INITIALIZE:
      robotMode = "Initializing";
      break;
    case OSDK_ROBOT_STATE_CAR:
      robotMode = "Car";
      break;
    case OSDK_ROBOT_STATE_STAND:
      robotMode = "Stand";
      break;
    case OSDK_ROBOT_STATE_TRANSITION_DOWN:
      robotMode = "Transition Up";
      break;
    case OSDK_ROBOT_STATE_TRANSITION_UP:
      robotMode = "Transition Down";
      break;
    default:
      robotMode = "Disconnect";
      break;
  }

  cout << "__status__\n";
  printf("ctrl mode:  %u\n", status.ctrl_mode);
  cout << "robot mode: " << robotMode << endl
       << "error: " << status.error << endl
       << "warning " << status.warning << endl;
}

void Telemetry::powerLog(const OSDK_Push_Data_Power_t & power)
{
  this->timeLog();
  printf(
    "__power__\nbattery voltage:\t%f\nbattery current:\t%f\nbattery percentage:\t%i\n",
    power.voltage, power.current, static_cast<int>(power.power_percent));
}

void Telemetry::quaternionLog(const OSDK_Push_Data_Quaternion_t & quaternion)
{
  this->timeLog();
  printf(
    "__quaternion__\nw:\t%f\nx:\t%f\ny:\t%f\nz:\t%f\n", quaternion.w, quaternion.x, quaternion.y,
    quaternion.z);
}

void Telemetry::acclLog(const OSDK_Push_Data_XYZ_t & accl)
{
  this->timeLog();
  cout << "__accleration__\n";
  cout << "x:\t" << accl.x << endl << "y:\t" << accl.y << endl << "z:\t" << accl.z << endl;
}

void Telemetry::gyroLog(const OSDK_Push_Data_XYZ_t & gyro)
{
  this->timeLog();
  cout << "__gyro__\n";
  cout << "x:\t" << gyro.x << endl << "y:\t" << gyro.y << endl << "z:\t" << gyro.z << endl;
}

void Telemetry::rcLog(const OSDK_Push_Data_RC_t & rc)
{
  this->timeLog();
  cout << "__rc__\n";
  cout << "frame_lost: " << rc.frame_lost << endl
       << "failsafe:   " << rc.failsafe << endl
       << "reserve:     " << rc.reserve << endl;
}

void Telemetry::motorLog(const Leg_Motors & motors)
{
  this->timeLog();
  cout << "__motors__\n";
  cout << "LH_enc_rev:  " << motors.left_hip.rev << endl
       << "LH_pos:  " << motors.left_hip.pos << endl
       << "LH_ang_vel:  " << motors.left_hip.vel << endl
       << "LH_iq:       " << motors.left_hip.iq << endl
       << "LK_enc_rev:  " << motors.left_knee.rev << endl
       << "LK_pos:  " << motors.left_knee.pos << endl
       << "LK_ang_vel:  " << motors.left_knee.vel << endl
       << "LK_iq:       " << motors.left_knee.iq << endl
       << "LW_enc_rev:  " << motors.left_wheel.rev << endl
       << "LW_pos:  " << motors.left_wheel.pos << endl
       << "LW_ang_vel:  " << motors.left_wheel.vel << endl
       << "LW_iq:       " << motors.left_wheel.iq << endl
       << "RH_enc_rev:  " << motors.right_hip.rev << endl
       << "RH_pos:  " << motors.right_hip.pos << endl
       << "RH_ang_vel:  " << motors.right_hip.vel << endl
       << "RH_iq:       " << motors.right_hip.iq << endl
       << "RK_enc_rev:  " << motors.right_knee.rev << endl
       << "RK_pos:  " << motors.right_knee.pos << endl
       << "RK_ang_vel:  " << motors.right_knee.vel << endl
       << "RK_iq:       " << motors.right_knee.iq << endl
       << "RW_ang_rev:  " << motors.right_wheel.rev << endl
       << "RW_pos:  " << motors.right_wheel.pos << endl
       << "RW_ang_vel:  " << motors.right_wheel.vel << endl
       << "RW_iq:       " << motors.right_wheel.iq << endl;
}
