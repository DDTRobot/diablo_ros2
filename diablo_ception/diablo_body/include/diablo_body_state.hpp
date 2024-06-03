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

#include <chrono>

#include "builtin_interfaces/msg/time.hpp"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"
#include "motion_msgs/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

class diablo_body_state_publisher
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  DIABLO::OSDK::Vehicle * vehicle;
  rclcpp::Node::SharedPtr node_ptr;
  motion_msgs::msg::RobotStatus robot_state_msg_;
  builtin_interfaces::msg::Time robot_state_timestamp;
  rclcpp::Publisher<motion_msgs::msg::RobotStatus>::SharedPtr robot_state_Publisher_;

public:
  diablo_body_state_publisher(rclcpp::Node::SharedPtr node_ptr, DIABLO::OSDK::Vehicle * vehicle);
  ~diablo_body_state_publisher() {}
  void body_pub_init(void);
  void lazyPublisher(void);
};
