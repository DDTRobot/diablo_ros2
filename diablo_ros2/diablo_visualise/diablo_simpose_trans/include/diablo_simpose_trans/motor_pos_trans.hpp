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

#ifndef DIABLO_SIMPOSE_TRANS__MOTOR_POS_TRANS_HPP_
#define DIABLO_SIMPOSE_TRANS__MOTOR_POS_TRANS_HPP_

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "motion_msgs/msg/leg_motors.hpp"
#include "motion_msgs/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MotorTransNode : public rclcpp::Node
{
public:
  explicit MotorTransNode(std::string name) : Node(name)
  {
    RCLCPP_INFO(this->get_logger(), "Transform motor msg 2 rviz node: %s.", name.c_str());
    sub_motor_msg_ = this->create_subscription<motion_msgs::msg::LegMotors>(
      "/diablo/sensor/Motors", 10,
      std::bind(&MotorTransNode::motor_callback, this, std::placeholders::_1));
    sub_robot_state_msg_ = this->create_subscription<motion_msgs::msg::RobotStatus>(
      "/diablo/sensor/Body_state", 10,
      std::bind(&MotorTransNode::robot_state_callback, this, std::placeholders::_1));
    pub_rviz_joint_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SystemDefaultsQoS());
  }

private:
  bool first_start_mark = true;              // Mark of first start, to init joint_angle_compensate.
  float position_map(float position_value);  // Map sdk original motor position to rviz joint.
  void motor_callback(const motion_msgs::msg::LegMotors::SharedPtr msg);
  void robot_state_callback(const motion_msgs::msg::RobotStatus::SharedPtr msg);

private:
  rclcpp::Subscription<motion_msgs::msg::LegMotors>::SharedPtr sub_motor_msg_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_rviz_joint_;
  rclcpp::Subscription<motion_msgs::msg::RobotStatus>::SharedPtr sub_robot_state_msg_;

  motion_msgs::msg::RobotStatus body_state_msg_;
  sensor_msgs::msg::JointState joint_state_msg_;
  builtin_interfaces::msg::Time motors_timestamp_;
};
#endif  // DIABLO_SIMPOSE_TRANS__MOTOR_POS_TRANS_HPP_
