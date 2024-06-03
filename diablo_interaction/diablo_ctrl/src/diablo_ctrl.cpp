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

#include "diablo_ctrl.hpp"

#include <iostream>

void diabloCtrlNode::Motion_callback(const motion_msgs::msg::MotionCtrl::SharedPtr msg)
{
  if (!pMovementCtrl->in_control()) {
    pMovementCtrl->obtain_control();
    return;
  }
  if (pMovementCtrl->ctrl_mode_data.height_ctrl_mode == 1) {
    pMovementCtrl->ctrl_data.up = 0.0f;
  }
  pMovementCtrl->ctrl_data.forward = 0.0f;
  pMovementCtrl->ctrl_data.left = 0.0f;

  switch (msg->cmd_id) {
    case CMD_GO_FORWARD:
      pMovementCtrl->ctrl_data.forward = msg->value;
      break;

    case CMD_GO_LEFT:
      pMovementCtrl->ctrl_data.left = msg->value;
      break;

    case CMD_ROLL_RIGHT:
      pMovementCtrl->ctrl_data.roll = msg->value;
      break;

    case CMD_STAND_UP:
      pMovementCtrl->SendTransformUpCmd();
      break;

    case CMD_STAND_DOWN:
      pMovementCtrl->SendTransformDownCmd();
      break;

    case CMD_HEIGH_MODE:
      pMovementCtrl->ctrl_mode_data.height_ctrl_mode = static_cast<int>(msg->value);
      pMovementCtrl->ctrl_mode_cmd = true;
      break;

    case CMD_BODY_UP:
      pMovementCtrl->ctrl_data.up = msg->value;
      break;

    case CMD_PITCH_MODE:
      pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = static_cast<int>(msg->value);
      pMovementCtrl->ctrl_mode_cmd = true;
      break;

    case CMD_PITCH:
      pMovementCtrl->ctrl_data.pitch = msg->value;
      break;

    case CMD_SPEED_MODE:
      pMovementCtrl->ctrl_mode_data.head_controller_mode = static_cast<int>(msg->value);

    default:
      break;
  }

  if (pMovementCtrl->ctrl_mode_cmd) {
    pMovementCtrl->SendMovementModeCtrlCmd();
  } else {
    pMovementCtrl->SendMovementCtrlCmd();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<diabloCtrlNode>("diablo_ctrl_node");

  DIABLO::OSDK::HAL_Pi Hal;
  if (Hal.init("/dev/ttyS3")) return -1;

  DIABLO::OSDK::Vehicle vehicle(&Hal);
  if (vehicle.init()) return -1;

  vehicle.telemetry->activate();

  diablo_imu_publisher imuPublisher(node, &vehicle);
  imuPublisher.imu_pub_init();

  diablo_battery_publisher batteryPublisher(node, &vehicle);
  batteryPublisher.battery_pub_init();

  diablo_motors_publisher motorsPublisher(node, &vehicle);
  motorsPublisher.motors_pub_init();

  diablo_body_state_publisher bodyStatePublisher(node, &vehicle);
  bodyStatePublisher.body_pub_init();

  // vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);
  // vehicle.telemetry->setMaxSpeed(1.0);
  node->pMovementCtrl = vehicle.movement_ctrl;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
