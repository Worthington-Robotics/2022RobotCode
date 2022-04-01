#pragma once

#include <std_msgs/msg/int16.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <frc/DoubleSolenoid.h>

#define PCMID 0

namespace solenoid
{

    class Solenoid
    {
    public:
        frc::DoubleSolenoid::Value state = frc::DoubleSolenoid::kReverse;

        Solenoid(frc::PneumaticsModuleType type, int highID, int lowID, const std::string& solenoidName)
        {
            solenoid = std::make_shared<frc::DoubleSolenoid>(PCMID, type, highID, lowID);
            name = solenoidName;
        }

        void set(const std::shared_ptr<std_msgs::msg::Int16> msg)
        {
            #ifdef PNU_DEBUG
                std::cout << name << " has been set to " << msg->data << std::endl;
            #endif
            if (msg->data < 0)
            {
                state = frc::DoubleSolenoid::kReverse;
            }
            else if (msg->data > 0)
            {
                state = frc::DoubleSolenoid::kForward;
            }
            else
            {
                state = frc::DoubleSolenoid::kOff;
            }
        }

        std::string getName(){
            return name + "_solenoid";
        }

        std::shared_ptr<frc::DoubleSolenoid> getSolenoid(){
            return solenoid;
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