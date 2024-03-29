#include "subsystems/Battery.h"

#include <iostream>
#include "frc/smartdashboard/SmartDashboard.h"

using std::placeholders::_1;

namespace robot {
    Battery::Battery() {
        
    }
    void Battery::createRosBindings(rclcpp::Node *node) {
        IdleStageSub = node->create_subscription<std_msgs::msg::Int16>("/battery/idle", rclcpp::SensorDataQoS(), std::bind(&Battery::idleStageCallback, this, _1));
        ResetStageSub = node->create_subscription<std_msgs::msg::Int16>("/battery/idle", rclcpp::SensorDataQoS(), std::bind(&Battery::idleStageCallback, this, _1));
        node->create_publisher<std_msgs::msg::Int16>("/battery/idle", rclcpp::SystemDefaultsQoS());
    }
    void Battery::reset() {
        
    }
    void Battery::onStart() {

    }
    void Battery::onLoop(double currentTime) {
        
    }
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
        double power = panel.GetTotalPower(); //power from panel
        double currentTime = frc::Timer::GetFPGATimestamp().to<double>();
        double timeDiff = currentTime - previousTime; //amount of time passed since last call
        double amt = power * timeDiff; //output
        powerUsed = previous + amt; 
        previous = amt;
        previousTime = currentTime;
        return amt;
    }
    void Battery::idleStageCallback(const std_msgs::msg::Int16 msg) {
        std::cout << "changing idle mode to " << msg.data << std::endl;
        idleStage = msg.data;
    }
    void Battery::updateSensorData() {

    }
}
