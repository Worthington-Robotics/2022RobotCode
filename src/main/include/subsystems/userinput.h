#pragma once

#include "subsystems/Subsystem.h"
#include "Util.h"

#include <frc/DriverStation.h>
#include <frc/Joystick.h>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <cmath>
#include <string>

namespace robot {

    class UserInput : public Subsystem {
    public:
    
        UserInput();

        void registerSticks(std::vector<int> stickIds);

        void createRosBindings(rclcpp::Node *node) override;

        void reset() override;

        void onStart() override;

        void onLoop(double currentTime) override;

        void publishData() override;

        void updateSensorData() override;

        /**
         * Evaluate the deadband on the joystick.
         * @param joyMsg the joystick message containing data to be parsed
         * @param deadBand the minimum value the joystick must have before it becomes non-zero
         * @param power the power to apply to the joystick as a ramping function (recommend 2 or 3)
         * @return the output values of the different joystick axes
         **/
        static std::vector<double> evalDeadband(const MSG_JOY &joyMsg,
                                                const double deadBand,
                                                const int power);

        /**
         * Get the deadbanded value of the joystick and apply a scalar factor to adjust the maximum range
         * @param joyMsg the joystick message containing data to be parsed
         * @param deadband the minimum value the joystick must have before it becomes non-zero
         * @param pow the power to apply to the joystick as a ramping function (recommend 2 or 3)
         * @param scalars the scalars to use on the associated axes. if an axis exists with no matching scalar,
         * it will be left un-processed
         * @return the output values of the different joystick axes
         **/
        static std::vector<double> scalarCut(const MSG_JOY &joyMsg,
                                            const double deadBand,
                                            const int power,
                                            const std::vector<double> scalars);

        /**
         * Maps a value bounded from [-1, 1] to [minOutput, maxOutput]
         * @param input the input value to map
         * @param minOutput the minimum output value aftermapping
         * @param maxOutput the maximimum output value after mapping 
         **/
        static double mapValue(double input, double minOutput, double maxOutput);

    private:
        std::vector<frc::Joystick> sticks = {};
        std::vector<ROS_PUB(MSG_JOY)> stickPubs = {};

        void setStickZero(MSG_JOY lastStick0);

        void setStickOne(MSG_JOY lastStick1);
        
        ROS_PUB(MSG_INT) intakeIndexerPub;
        bool intakeIndexerMsgUpdate = false, intakeIndexerPressed = false;

        ROS_PUB(MSG_INT) intakeSolePub;
        bool intakeSoleMsgUpdate = false, intakeSolePressed = false;

        ROS_PUB(MSG_INT) flyWheelModePub;
        bool flywheelModeUpdate = false, flywheelModePressed = false;
    };

} // namespace robot
