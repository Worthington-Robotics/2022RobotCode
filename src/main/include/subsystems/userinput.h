#pragma once
#include "subsystems/Subsystem.h"
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <vector>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot
{

    class UserInput : public Subsystem
    {
    public:
        UserInput();

        void registerSticks(std::vector<int> stickIds);

        /**
         * Override this function in order to create pulbishers or subscribers against the parent node.
         * NOTE: This function is automatically called by the subsystem manager on registration
         **/
        void createRosBindings(rclcpp::Node *node) override;

        /**
         * Override this function with all the nessecary code needed to reset a subsystem.
         * For sticks, nothing needs to be reset (no accumulators)
         **/
        void reset() override;

        /**
         * Overrride this function with any code needed to be called only once on the first onloop iteration
         * For sticks, nothing is called during onstart
         **/
        void onStart() override;

        /**
         * Override this function for any code that must be called periodically by the subsystem
         * For sticks, nothing is called during onloop
         **/
        void onLoop(double currentTime) override;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void publishData() override;
        
        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void updateSensorData() override;
        

        /**
         * Evaluate the deadband on the joystick.
         * @param joyMsg the joystick message containing data to be parsed
         * @param deadBand the minimum value the joystick must have before it becomes non-zero
         * @param power the power to apply to the joystick as a ramping function (recommend 2 or 3)
         * @return the output values of the different joystick axes
         **/
        static std::vector<double> evalDeadband(const sensor_msgs::msg::Joy &joyMsg,
                                                const double deadBand, const int power);

        /**
         * Get the deadbanded value of the joystick and apply a scalar factor to adjust the maximum range
         * @param joyMsg the joystick message containing data to be parsed
         * @param deadband the minimum value the joystick must have before it becomes non-zero
         * @param pow the power to apply to the joystick as a ramping function (recommend 2 or 3)
         * @param scalars the scalars to use on the associated axes. if an axis exists with no matching scalar,
         * it will be left un-processed
         * @return the output values of the different joystick axes
         **/
        static std::vector<double> scalarCut(const sensor_msgs::msg::Joy &joyMsg,
                                             const double deadBand, const int power, const std::vector<double> scalars);

        /**
         * Maps a value bounded from [-1, 1] to [minOutput, maxOutput]
         * @param input the input value to map
         * @param minOutput the minimum output value aftermapping
         * @param maxOutput the maximimum output value after mapping 
         **/
        static double mapValue(double input, double minOutput, double maxOutput);

        void enableDebug(bool debugEnable) override {

        }

    private:
        std::vector<frc::Joystick> sticks = {};
        std::vector<rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr> stickPubs = {};
    };

} // namespace robot
