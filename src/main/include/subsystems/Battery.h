#pragma once

#include "subsystems/Subsystem.h"
#include "Constants.h"
#include "Util.h"

#include <rclcpp/rclcpp.hpp>
#include <frc/Timer.h>
#include <frc/DriverStation.h>

#include <frc/PowerDistribution.h>
#include <frc/RobotController.h>

#include <std_msgs/msg/int16.hpp>

namespace robot {

    class Battery : public Subsystem {
    public:

        Battery();

        void createRosBindings(rclcpp::Node *node) override;

        void reset() override;

        void onStart() override;

        void onLoop(double currentTime) override;

        void publishData() override;

        /* Callbacks for ROS Subscribers */

        void idleStageCallback(const MSG_INT msg);
        void resetStageCallback();

        /* Reset idle countdown */
        void resetIdle();
        
    private:

        void execActions();

        void updateSensorData();

        /* Get the amount of power the robot has used since the last call */
        double getPowerUsage();

        int idleTime = 0; /* Amount of time spent since last update */
        int idleStage = 0; /* Stage from 0-2 of current level of idle */

        double previous = 0.0; /* Result of the last getPowerUsage() call */
        double previousTime = 0.0; /* Timestamp of the last getPowerUsage() call */
        double previousPower = 18.0; /* Level of the battery on the last update */
        double maxPower = 18.0; /* Out of 18.8, 18, and 17, this is a general average    should probably be changed later */
        double batteryPower = 18.0; /* Amount of power left in the battery */
        double powerUsed = 0.0; /* Average (ish) power usage up until the current time */

        frc::PowerDistribution panel{0, frc::PowerDistribution::ModuleType::kCTRE};

        /* ROS Subscibers */

        ROS_SUB(MSG_INT) IdleStageSub;
        ROS_SUB(MSG_INT) ResetStageSub;
    };
} // namespace robot
