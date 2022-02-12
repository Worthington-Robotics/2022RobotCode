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

        virtual void declareConfig(std::shared_ptr<rclcpp::Node>){}

        virtual void executeConfig(std::shared_ptr<rclcpp::Node>){}

        virtual void setValue(std::shared_ptr<can_msgs::msg::MotorMsg> msg) {}

        virtual void configMotorPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
                         std::shared_ptr<can_msgs::srv::SetPIDFGains::Response> resp){}

        virtual JointState getJointState() {}

    };

    struct MotorContainer {
        Motor& motor;
        rclcpp::Subscription<can_msgs::msg::MotorMsg>::SharedPtr sub;
        rclcpp::Service<can_msgs::srv::SetPIDFGains>::SharedPtr pidfSrv;
    };
} // namespace motors
