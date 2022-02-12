#include <can_msgs/msg/motor_msg.hpp>
#include <can_msgs/srv/set_pidf_gains.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <cctype>
#include <exception>
#include <memory>
#include <string>

#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "robot_lib/motor_abs/motor.hpp"
#include "robot_lib/motor_abs/talon_brushless.hpp"

#define SAFETY_TIMEOUT_MS 100
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class HardwareNode : public rclcpp::Node {
 public:

    void createMotors() {
        std::vector<std::string> motorNames;
        RCLCPP_DEBUG(get_logger(), "Loading motors from config");
        try {
            this->declare_parameter<std::vector<std::string>>("motor_names", {});
            motorNames = this->get_parameter("motor_names").as_string_array();
        } catch (std::runtime_error) {
            // This is what happens if the array is empty
            RCLCPP_FATAL(get_logger(), "No motors declared in config. Controller will now exit!");
            throw std::runtime_error("No motors declared in config. Controller will now exit!");
        }

        if (motorNames.size() < 1) {
            RCLCPP_FATAL(get_logger(), "No motors declared in config. Controller will now exit!");
            throw std::runtime_error("No motors declared in config. Controller will now exit!");
        }

        for (auto motorName : motorNames) {
            RCLCPP_DEBUG_STREAM(get_logger(), "Building motor " << motorName);
            this->declare_parameter<int>(motorName + ".id", 0);
            this->declare_parameter<std::string>(motorName + ".type", "");

            auto type = this->get_parameter(motorName + ".type").as_string();
            std::transform(type.begin(), type.end(), type.begin(),
                           [](unsigned char c) { return std::tolower(c); });

            motors::Motor *motorBase;
            if (type.find("talonfx") != std::string::npos) {
                motorBase = new motors::TalonBrushless(
                    this->get_parameter(motorName + ".id").as_int(),
                    motorName);
            } else {
                RCLCPP_ERROR_STREAM(get_logger(), "Error loading motor " << motorName << " with type " << type);
                continue;
            }

            RCLCPP_DEBUG_STREAM(get_logger(), "Binding motor " << motorName);
            motors::MotorContainer motorContainer = {
                *motorBase,
                create_subscription<can_msgs::msg::MotorMsg>(
                    "motor/" + motorName, rclcpp::SystemDefaultsQoS(),
                    std::bind(&motors::Motor::setValue, motorContainer.motor, _1), subsOpt),
                create_service<can_msgs::srv::SetPIDFGains>("motor/" + motorName + "/set_pidf",
                                                            std::bind(&motors::Motor::configMotorPIDF, motorContainer.motor, _1, _2))};

            motorContainers.push_back(motorContainer);
        }

        RCLCPP_DEBUG(get_logger(), "Declaring motor parameters");
        for (auto motorContainer : motorContainers) {
            this->shared_from_this();
            motorContainer.motor.declareConfig(this->shared_from_this());
        }

        RCLCPP_DEBUG(get_logger(), "Configuring motors from parameters");
        for (auto motorContainer : motorContainers) {
            motorContainer.motor.executeConfig(this->shared_from_this());
        }

        // Allow timer to free run now
        highRate->reset();
    }
};
