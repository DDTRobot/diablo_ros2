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
<<<<<<< HEAD
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "ception_msgs/msg/imu_euler.hpp"
#include <builtin_interfaces/msg/time.hpp>
=======

#include "builtin_interfaces/msg/time.hpp"
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class diablo_imu_publisher
{
private:
<<<<<<< HEAD
    rclcpp::TimerBase::SharedPtr                                        timer_;
    DIABLO::OSDK::Vehicle*                                             vehicle;
    sensor_msgs::msg::Imu                                             imu_msg_;
    ception_msgs::msg::IMUEuler                                     euler_msg_;

    rclcpp::Node::SharedPtr                                           node_ptr;
    builtin_interfaces::msg::Time                                imu_timestamp;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_Publisher_;
    rclcpp::Publisher<ception_msgs::msg::IMUEuler>::SharedPtr euler_Publisher_;

public:
    diablo_imu_publisher(rclcpp::Node::SharedPtr node_ptr,DIABLO::OSDK::Vehicle* vehicle);
    // void toEulerAngle(void);
    ~diablo_imu_publisher(){}
    void imu_pub_init(void);
    void lazyPublisher(void);
};



=======
  rclcpp::TimerBase::SharedPtr timer_;
  DIABLO::OSDK::Vehicle * vehicle;
  sensor_msgs::msg::Imu imu_msg_;
  rclcpp::Node::SharedPtr node_ptr;
  builtin_interfaces::msg::Time imu_timestamp;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_Publisher_;

public:
  diablo_imu_publisher(rclcpp::Node::SharedPtr node_ptr, DIABLO::OSDK::Vehicle * vehicle);
  ~diablo_imu_publisher() {}
  void imu_pub_init(void);
  void lazyPublisher(void);
};
>>>>>>> 67a684b70db424c49caea5813a47e0e0359b4918
