#pragma once

#include <can_msgs/msg/motor_msg.hpp>
#include <can_msgs/srv/set_pidf_gains.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace motors {
    struct JointState {
        std::string name;
        double position, velocity, effort;
    };

    class Motor {
    public:

        virtual void setValue(const std::shared_ptr<can_msgs::msg::MotorMsg> msg) {
            std::cout << "If you see this message you did not bind to this object with std::ref" << std::endl;
        }

        virtual void configMotorPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
                         std::shared_ptr<can_msgs::srv::SetPIDFGains::Response> resp) = 0;

        virtual JointState getJointState() {}

    };

    struct MotorContainer {
        Motor &motor;
        rclcpp::Subscription<can_msgs::msg::MotorMsg>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
        rclcpp::Service<can_msgs::srv::SetPIDFGains>::SharedPtr pidfSrv;
        bool shutUp = false;
    };
} // namespace motors