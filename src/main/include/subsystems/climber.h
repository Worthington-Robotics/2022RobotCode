#pragma once

#include "subsystems/Subsystem.h"
#include "Constants.h"
#include "Util.h"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <can_msgs/msg/motor_msg.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <vector>

namespace robot {

    class Climber : public Subsystem {
    public:
        Climber();

        void reset() {}

        void onStart() {}

        void onLoop(double currentTime) {}

        void updateSensorData() {}

        void createRosBindings(rclcpp::Node *node) override;

        void publishData() override;

    private:
        bool enableSole = false; 
        bool solePressed = false; 
        bool soleState = false;

        ROS_SUB(sensor_msgs::msg::Joy) stick1Sub;
        std::vector<rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr> climberEnabledSubs;
        std::vector<rclcpp::Publisher<can_msgs::msg::MotorMsg>::SharedPtr> climberDemandsPubs;
        ROS_PUB(std_msgs::msg::Int16) soleStatePub;
        std::vector<bool> climberEnabled {false, false}; 
        std::vector<double> climberDemands {0, 0}; 

        void setLClimberEnabled(const std_msgs::msg::Int16 msg) {
            climberEnabled.at(0) = (msg.data == 1);
        }

        void setRClimberEnabled(const std_msgs::msg::Int16 msg) {
            climberEnabled.at(1) = (msg.data == 1);
        }

        void setStick1Input(const sensor_msgs::msg::Joy msg) {
            climberDemands.at(0) = -msg.axes.at(1);
            climberDemands.at(1) = -msg.axes.at(1);
            climberEnabled.at(0) = msg.buttons.at(8) || msg.buttons.at(11);
            climberEnabled.at(1) = msg.buttons.at(9) || msg.buttons.at(11);
            enableSole = msg.buttons.at(10);
        }
    };

}