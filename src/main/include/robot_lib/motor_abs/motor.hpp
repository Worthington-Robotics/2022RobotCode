#pragma once

#include <can_msgs/msg/motor_msg.hpp>
#include <can_msgs/srv/set_pidf_gains.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace motors
{
    struct JointState {
        std::string name;
        double position, velocity, effort;
    };

    class Motor{
        public:

        virtual void setValue(std::shared_ptr<can_msgs::msg::MotorMsg> msg) {}

        virtual void configMotorPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
                         std::shared_ptr<can_msgs::srv::SetPIDFGains::Response> resp){}

        virtual JointState getJointState() {}

    };

    struct MotorContainer {
        std::shared_ptr<Motor> motor;
        rclcpp::Subscription<can_msgs::msg::MotorMsg>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
        rclcpp::Service<can_msgs::srv::SetPIDFGains>::SharedPtr pidfSrv;
    };
} // namespace motors
