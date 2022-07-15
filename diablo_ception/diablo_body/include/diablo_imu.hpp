#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_imu_publisher
{
private:

    rclcpp::TimerBase::SharedPtr                                timer_;
    DIABLO::OSDK::Vehicle*                                     vehicle;
    sensor_msgs::msg::Imu                                     imu_msg_;
    rclcpp::Node::SharedPtr                                   node_ptr;
    builtin_interfaces::msg::Time                        imu_timestamp;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_Publisher_;

public:
    diablo_imu_publisher(rclcpp::Node::SharedPtr node_ptr,DIABLO::OSDK::Vehicle* vehicle);
    ~diablo_imu_publisher(){}
    void imu_pub_init(void);
    void lazyPublisher(void);
};


