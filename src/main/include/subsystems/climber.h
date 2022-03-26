#pragma once

#include "subsystems/Subsystem.h"
#include "Constants.h"

// msgs used in this package
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <can_msgs/msg/motor_msg.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <vector>



namespace robot
{

    class Climber : public Subsystem
    {
    public:
        Climber();

        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/ 
        void reset() {}

        /**
         * Overrride this function with any code needed to be called only once on the first onloop iteration
         **/ 
        void onStart() {}

        /**
         * Override this function for any code that must be called periodically by the subsystem
         **/ 
        void onLoop(double currentTime) {}

        /**
         * Override this function with code needed to update sensors assosiated with the robot
         **/ 
        void updateSensorData() {}


        /**
         * Override this function in order to create pulbishers or subscribers against the parent node.
         * NOTE: This function is automatically called by the subsystem manager on registration
         **/
        void createRosBindings(rclcpp::Node *node) override;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void publishData() override;

    private:
        bool enableSole = false; 
        bool solePressed = false; 
        bool soleState = false;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr stick1Sub;
        std::vector<rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr> climberEnabledSubs;
        std::vector<rclcpp::Publisher<can_msgs::msg::MotorMsg>::SharedPtr> climberDemandsPubs;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr soleStatePub;
        std::vector<bool> climberEnabled {false, false}; 
        std::vector<double> climberDemands {0, 0}; 

        void setLClimberEnabled(const std_msgs::msg::Int16 msg) {
            if(msg.data == 1)
                climberEnabled.at(0) = true;
            else
                climberEnabled.at(0) = false;
        }

        void setRClimberEnabled(const std_msgs::msg::Int16 msg) {
            if(msg.data == 1)
                climberEnabled.at(1) = true;
            else
                climberEnabled.at(1) = false;
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