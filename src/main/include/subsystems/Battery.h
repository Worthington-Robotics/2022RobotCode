#pragma once

#include "subsystems/Subsystem.h"
#include <rclcpp/rclcpp.hpp>

#include "Constants.h"
#include "frc/Timer.h"
#include "frc/DriverStation.h"

#include "frc/PowerDistribution.h"
#include "frc/RobotController.h"

#include <std_msgs/msg/int16.hpp>

namespace robot
{

    class Battery : public Subsystem
    {
        public:

        Battery();

        /**
         * Override this function in order to create pulbishers or subscribers against the parent node.
         * NOTE: This function is automatically called by the subsystem manager on registration
         **/
        void createRosBindings(rclcpp::Node *node) override;

        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/
        void reset() override;

        /**
         * Overrride this function with any code needed to be called only once on the first onloop iteration
         **/
        void onStart() override;

        /**
         * Override this function for any code that must be called periodically by the subsystem
         **/
        void onLoop() override;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void publishData() override;

        /**
         * Callbacks for ROS Subscribers 
         **/

        void idleStageCallback(const std_msgs::msg::Int16 msg);

        /**
         * Reset idle countdown
         **/
        void resetIdle();
        
        int idleStage = 0; //stage from 0-2 of current level of idle

    private:

        void execActions();

        void updateSensorData();

        /**
         * Get the amount of power the robot has used since the last call
         **/
        double getPowerUsage();

        int idleTime = 0; //amount of time spent since last update

        double previous = 0.0; //result of the last getPowerUsage() call
        double previousTime = 0.0; //timestamp of the last getPowerUsage() call
        double previousPower = 18.0; //level of the battery on the last update
        double maxPower = 18.0; //out of 18.8, 18, and 17, this is a general average    should probably be changed later
        double batteryPower = 18.0; //amount of power left in the battery
        double powerUsed = 0.0; //average (ish) power usage up until the current time

        frc::PowerDistribution panel{0, frc::PowerDistribution::ModuleType::kCTRE};

        // ROS Subscibers
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr IdleStageSub;
    };
} // namespace robot
