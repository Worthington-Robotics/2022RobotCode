#include "subsystems/Battery.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

using std::placeholders::_1;

namespace robot {
    
    Battery::Battery() {}

    void Battery::createRosBindings(rclcpp::Node *node) {
        IdleStageSub = node->create_subscription<IntMsg>("/battery/idle", SENSOR_QOS, std::bind(&Battery::idleStageCallback, this, _1));
        ResetStageSub = node->create_subscription<IntMsg>("/battery/idle", SENSOR_QOS, std::bind(&Battery::idleStageCallback, this, _1));
        node->create_publisher<IntMsg>("/battery/idle", DEFAULT_QOS);
    }

    void Battery::reset() {}

    void Battery::onStart() {}

    void Battery::onLoop(double currentTime) {}

    void Battery::publishData() {
        int time1 = 5 * 100;
        int time2 = 15 * 100;
        idleTime++;
        if (idleTime >= time1) {
            if (idleTime >= time2) {
                idleStage = 1;
            } else {
                idleStage = 2;
            }
        } else {
            idleStage = 0;
        }

    }

    void Battery::resetIdle() {
        idleTime = 0;
        idleStage = 0;
    }

    double Battery::getPowerUsage() { 
        double power = panel.GetTotalPower(); /* Power from panel */
        double currentTime = GET_TIME_DOUBLE;
        double timeDiff = currentTime - previousTime; /* Amount of time passed since last call */
        double amt = power * timeDiff; /* Output */
        powerUsed = previous + amt; 
        previous = amt;
        previousTime = currentTime;
        return amt;
    }

    void Battery::idleStageCallback(const IntMsg msg) {
        std::cout << "changing idle mode to " << msg.data << std::endl;
        idleStage = msg.data;
    }

    void Battery::updateSensorData() {}

} // namespace robot