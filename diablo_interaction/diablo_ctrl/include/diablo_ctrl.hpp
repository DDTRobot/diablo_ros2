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

#include <string>

#include "diablo_battery.hpp"
#include "diablo_body_state.hpp"
#include "diablo_imu.hpp"
#include "diablo_legmotors.hpp"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "rclcpp/rclcpp.hpp"

#define CMD_GO_FORWARD 0x08
#define CMD_GO_LEFT 0x04
#define CMD_ROLL_RIGHT 0x09

#define CMD_HEIGH_MODE 0x01  // set 0 or 1
#define CMD_BODY_UP 0x11

#define CMD_STAND_UP 0x02
#define CMD_STAND_DOWN 0x12

#define CMD_PITCH 0x03
#define CMD_PITCH_MODE 0x13

#define CMD_SPEED_MODE 0x05

class diabloCtrlNode : public rclcpp::Node
{
public:
<<<<<<< HEAD
    diabloCtrlNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Sub node: %s.",name.c_str());
        sub_movement_cmd = this->create_subscription<motion_msgs::msg::MotionCtrl>("diablo/MotionCmd", 10, std::bind(&diabloCtrlNode::Motion_callback, this, std::placeholders::_1));
        ctrl_msg_.value.up = 1.0;
    }
    ~diabloCtrlNode();

    void run_(void);
    void heart_beat_loop(void);
    std::shared_ptr<std::thread> thread_;
    DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;
    DIABLO::OSDK::Telemetry* pTelemetry;




private:
    void Motion_callback(const motion_msgs::msg::MotionCtrl::SharedPtr msg);

private:
    rclcpp::Subscription<motion_msgs::msg::MotionCtrl>::SharedPtr sub_movement_cmd;
    OSDK_Movement_Ctrl_t    cmd_value;
    bool                onSend = true;
    bool        thd_loop_mark_ = true;
    motion_msgs::msg::MotionCtrl                                       ctrl_msg_;
};


=======
  explicit diabloCtrlNode(std::string name) : Node(name)
  {
    RCLCPP_INFO(this->get_logger(), "Sub node: %s.", name.c_str());
    sub_movement_cmd = this->create_subscription<motion_msgs::msg::MotionCtrl>(
      "diablo/MotionCmd", 10,
      std::bind(&diabloCtrlNode::Motion_callback, this, std::placeholders::_1));
  }

  DIABLO::OSDK::Movement_Ctrl * pMovementCtrl;

private:
  void Motion_callback(const motion_msgs::msg::MotionCtrl::SharedPtr msg);

private:
  rclcpp::Subscription<motion_msgs::msg::MotionCtrl>::SharedPtr sub_movement_cmd;
};
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918
