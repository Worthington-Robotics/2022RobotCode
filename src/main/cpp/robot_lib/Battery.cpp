#include "robot_lib/Battery.h"
#include <iostream>

namespace robot
{
    Battery::Battery(){

    }
    double Battery::GetPowerUsage() { //get amount of power used over time from the PD panel
        double power = frc::RobotController::GetInputCurrent() * frc::RobotController::GetInputVoltage(); //power in joules from panel
        double currentTime = frc::Timer::GetFPGATimestamp().to<double>();
        double timeDiff = currentTime - previousTime; //amount of time passed since last call
        double amt = power * timeDiff; //output
        powerUsed = previous + amt; 
        previous = amt;
        previousTime = currentTime;
        return amt;
    }
} // namespace robot