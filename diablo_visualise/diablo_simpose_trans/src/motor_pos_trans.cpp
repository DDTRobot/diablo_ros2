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

#include "diablo_simpose_trans/motor_pos_trans.hpp"

float MotorTransNode::position_map(float position_value)
{
  float map_value = (position_value > 0 ? 1 : -1) * position_value - 3.14;
  return map_value;
}

void MotorTransNode::robot_state_callback(const motion_msgs::msg::RobotStatus::SharedPtr msg)
{
  this->body_state_msg_.ctrl_mode_msg = msg->ctrl_mode_msg;
  this->body_state_msg_.robot_mode_msg = msg->robot_mode_msg;
  this->body_state_msg_.warning_msg = msg->warning_msg;
  this->body_state_msg_.error_msg = msg->error_msg;
}

void MotorTransNode::motor_callback(const motion_msgs::msg::LegMotors::SharedPtr msg)
{
  motors_timestamp_ = this->get_clock()->now();
  joint_state_msg_.header.stamp = motors_timestamp_;

  // Use cosines law to calculate Rev3 angle. l_a = l_b = 1.4 dm l_c = leg_length * 10
  float l_angleB =
    (3.14 - acos((3.92 - ((msg->left_leg_length * 10) * (msg->left_leg_length * 10))) / 3.92)) / 2;
  float r_angleB =
    (3.14 - acos((3.92 - ((msg->right_leg_length * 10) * (msg->right_leg_length * 10))) / 3.92)) /
    2;

  float joint_angle_compensate;
  if (this->body_state_msg_.robot_mode_msg == 3) {
    joint_angle_compensate = 1.0;
  } else {
    joint_angle_compensate = 1.9;
  }

  joint_state_msg_.name = {"Rev1", "Rev2", "Rev3", "Rev4", "Rev5", "Rev6", "Rev7", "Rev8"};
  joint_state_msg_.position = {
    position_map(msg->right_hip_pos),
    position_map(msg->left_hip_pos),
    (-l_angleB) - joint_angle_compensate,
    (2 * l_angleB),
    position_map(msg->left_wheel_pos),
    (r_angleB) + joint_angle_compensate,
    -(2 * r_angleB),
    position_map(msg->right_wheel_pos),
  };
  pub_rviz_joint_->publish(joint_state_msg_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto pos_trans_node = std::make_shared<MotorTransNode>("motor_trans_node");

  rclcpp::spin(pos_trans_node);
  rclcpp::shutdown();
  return 0;
}
