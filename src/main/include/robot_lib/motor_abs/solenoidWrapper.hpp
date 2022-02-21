#pragma once

#include <std_msgs/msg/int16.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <frc/DoubleSolenoid.h>

namespace solenoid
{

    class Solenoid
    {
    public:
        Solenoid(frc::PneumaticsModuleType type, int highID, int lowID, const std::string& solenoidName)
        {
            solenoid = std::make_shared<frc::DoubleSolenoid>(type, highID, lowID);
            name = solenoidName;
        }

        void set(const std::shared_ptr<std_msgs::msg::Int16> msg)
        {
            if (msg->data < 0)
            {
                solenoid->Set(frc::DoubleSolenoid::kReverse);
            }
            else if (msg->data > 0)
            {
                solenoid->Set(frc::DoubleSolenoid::kForward);
            }
            else
            {
                solenoid->Set(frc::DoubleSolenoid::kOff);
            }
        }

        std::string getName(){
            return name + "_solenoid";
        }

    private:
        std::string name;
        std::shared_ptr<frc::DoubleSolenoid> solenoid;
    };

    struct SolenoidContainer
    {
        Solenoid solenoid;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub;
    };
} // namespace motors