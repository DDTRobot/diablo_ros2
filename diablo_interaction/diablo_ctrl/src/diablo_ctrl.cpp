<<<<<<< HEAD
#include <iostream>
#include "diablo_ctrl.hpp"

using namespace std;

void diabloCtrlNode::heart_beat_loop(void){
    RCLCPP_INFO(this->get_logger(), "start.");
    while (this->thd_loop_mark_)
    {
        if (onSend)
        {
            if(!pMovementCtrl->in_control())
            {
                pMovementCtrl->obtain_control();
                continue;
            }
            
            if(!ctrl_msg_.mode_mark){
                pMovementCtrl->ctrl_data.forward = ctrl_msg_.value.forward;
                pMovementCtrl->ctrl_data.left = ctrl_msg_.value.left;
                pMovementCtrl->ctrl_data.up = ctrl_msg_.value.up;
                pMovementCtrl->ctrl_data.roll = ctrl_msg_.value.roll;
                pMovementCtrl->ctrl_data.pitch = ctrl_msg_.value.pitch;
                pMovementCtrl->ctrl_data.leg_split = ctrl_msg_.value.leg_split;
                pMovementCtrl->SendMovementCtrlCmd();
                
            }else{
                if(ctrl_msg_.mode.stand_mode)
                    pMovementCtrl->SendTransformUpCmd();
                else{
                    pMovementCtrl->SendTransformDownCmd();
                }
                pMovementCtrl->ctrl_mode_data.height_ctrl_mode = ctrl_msg_.mode.height_ctrl_mode;
                pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = ctrl_msg_.mode.pitch_ctrl_mode;
                pMovementCtrl->ctrl_mode_data.roll_ctrl_mode = ctrl_msg_.mode.roll_ctrl_mode;
                pMovementCtrl->SendMovementModeCtrlCmd();
            }
            usleep(180000);
        }
    }
}

void diabloCtrlNode::run_(void){
    this->thd_loop_mark_ = true;
    this->thread_ = std::make_shared<std::thread>(&diabloCtrlNode::heart_beat_loop,this);
}

void diabloCtrlNode::Motion_callback(const motion_msgs::msg::MotionCtrl::SharedPtr msg)
{
    onSend = false;
    if(!pMovementCtrl->in_control())
    {
        pMovementCtrl->obtain_control();
        return;
    }
    ctrl_msg_.mode = msg->mode;
    ctrl_msg_.mode_mark = msg->mode_mark;
    ctrl_msg_.value = msg->value;
    
    if(!msg->mode_mark){
        pMovementCtrl->ctrl_data.forward = msg->value.forward;
        pMovementCtrl->ctrl_data.left = msg->value.left;
        pMovementCtrl->ctrl_data.up = msg->value.up;
        pMovementCtrl->ctrl_data.roll = msg->value.roll;
        pMovementCtrl->ctrl_data.pitch = msg->value.pitch;
        pMovementCtrl->ctrl_data.leg_split = msg->value.leg_split;
        pMovementCtrl->SendMovementCtrlCmd();
    }else{
 
        if(msg->mode.stand_mode)
            pMovementCtrl->SendTransformUpCmd();
        else{
            pMovementCtrl->SendTransformDownCmd();
        }
        
        if(pTelemetry->status.robot_mode == 3){
            pMovementCtrl->SendJumpCmd(msg->mode.jump_mode);
        }
        pMovementCtrl->SendDanceCmd(msg->mode.split_mode);
        pMovementCtrl->ctrl_mode_data.height_ctrl_mode = msg->mode.height_ctrl_mode;
        pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = msg->mode.pitch_ctrl_mode;
        pMovementCtrl->ctrl_mode_data.roll_ctrl_mode = msg->mode.roll_ctrl_mode;
        pMovementCtrl->SendMovementModeCtrlCmd();
    }
    onSend = true;
}



diabloCtrlNode::~diabloCtrlNode()
{
    RCLCPP_INFO(this->get_logger(), "done.");
    this->thd_loop_mark_ = false;
    thread_->join();
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<diabloCtrlNode>("diablo_ctrl_node");

    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init("/dev/ttyS3")) return -1;
=======
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
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918

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

<<<<<<< HEAD
    // vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);
    // vehicle.telemetry->setMaxSpeed(1.0);
    node->pMovementCtrl = vehicle.movement_ctrl;
    node->pTelemetry = vehicle.telemetry;
    node->run_();
=======
  diablo_body_state_publisher bodyStatePublisher(node, &vehicle);
  bodyStatePublisher.body_pub_init();
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918

  // vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);
  // vehicle.telemetry->setMaxSpeed(1.0);
  node->pMovementCtrl = vehicle.movement_ctrl;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
