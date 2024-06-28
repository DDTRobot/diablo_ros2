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

#include "diablo_legmotors.hpp"

using namespace std::chrono;

void diablo_motors_publisher::motors_pub_init(void)
{
<<<<<<< HEAD
    motors_Publisher_ = this->node_ptr->create_publisher<motion_msgs::msg::LegMotors>("diablo/sensor/Motors",10);
    timer_ = this->node_ptr->create_wall_timer(20ms,std::bind(&diablo_motors_publisher::lazyMotorsPublisher, this));
    this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_50Hz);
    this->vehicle->telemetry->configUpdate(); 
=======
  motors_Publisher_ =
    this->node_ptr->create_publisher<motion_msgs::msg::LegMotors>("diablo/sensor/Motors", 10);
  timer_ = this->node_ptr->create_wall_timer(
    100ms, std::bind(&diablo_motors_publisher::lazyMotorsPublisher, this));
  this->vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_10Hz);
  this->vehicle->telemetry->configUpdate();
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918
}

void diablo_motors_publisher::lazyMotorsPublisher(void)
{
  if (motors_Publisher_->get_subscription_count() > 0) {
    bool motors_Pub_mark = false;

    if (this->vehicle->telemetry->newcome & 0x01) {
      motors_Pub_mark = true;
      motors_msg_.left_hip_enc_rev = this->vehicle->telemetry->motors.left_hip.rev;
      motors_msg_.left_hip_pos = this->vehicle->telemetry->motors.left_hip.pos;
      motors_msg_.left_hip_vel = this->vehicle->telemetry->motors.left_hip.vel;
      motors_msg_.left_hip_iq = this->vehicle->telemetry->motors.left_hip.iq;

      motors_msg_.left_knee_enc_rev = this->vehicle->telemetry->motors.left_knee.rev;
      motors_msg_.left_knee_pos = this->vehicle->telemetry->motors.left_knee.pos;
      motors_msg_.left_knee_vel = this->vehicle->telemetry->motors.left_knee.vel;
      motors_msg_.left_knee_iq = this->vehicle->telemetry->motors.left_knee.iq;

      motors_msg_.left_wheel_enc_rev = this->vehicle->telemetry->motors.left_wheel.rev;
      motors_msg_.left_wheel_pos = this->vehicle->telemetry->motors.left_wheel.pos;
      motors_msg_.left_wheel_vel = this->vehicle->telemetry->motors.left_wheel.vel;
      motors_msg_.left_wheel_iq = this->vehicle->telemetry->motors.left_wheel.iq;

      motors_msg_.right_hip_enc_rev = this->vehicle->telemetry->motors.right_hip.rev;
      motors_msg_.right_hip_pos = this->vehicle->telemetry->motors.right_hip.pos;
      motors_msg_.right_hip_vel = this->vehicle->telemetry->motors.right_hip.vel;
      motors_msg_.right_hip_iq = this->vehicle->telemetry->motors.right_hip.iq;

      motors_msg_.right_knee_enc_rev = this->vehicle->telemetry->motors.right_knee.rev;
      motors_msg_.right_knee_pos = this->vehicle->telemetry->motors.right_knee.pos;
      motors_msg_.right_knee_vel = this->vehicle->telemetry->motors.right_knee.vel;
      motors_msg_.right_knee_iq = this->vehicle->telemetry->motors.right_knee.iq;

      motors_msg_.right_wheel_enc_rev = this->vehicle->telemetry->motors.right_wheel.rev;
      motors_msg_.right_wheel_pos = this->vehicle->telemetry->motors.right_wheel.pos;
      motors_msg_.right_wheel_vel = this->vehicle->telemetry->motors.right_wheel.vel;
      motors_msg_.right_wheel_iq = this->vehicle->telemetry->motors.right_wheel.iq;

      motors_msg_.left_leg_length = this->vehicle->telemetry->left_leg_length;
      motors_msg_.right_leg_length = this->vehicle->telemetry->right_leg_length;

      this->vehicle->telemetry->eraseNewcomeFlag(0xFE);
    }
    if (motors_Pub_mark) {
      motors_timestamp = this->node_ptr->get_clock()->now();
      motors_msg_.header.stamp = motors_timestamp;
      motors_msg_.header.frame_id = "diablo_robot";
      motors_Publisher_->publish(motors_msg_);
      motors_Pub_mark = false;
    }
  }
}

diablo_motors_publisher::diablo_motors_publisher(
  rclcpp::Node::SharedPtr node_ptr, DIABLO::OSDK::Vehicle * vehicle)
{
  this->node_ptr = node_ptr;
  this->vehicle = vehicle;
}
