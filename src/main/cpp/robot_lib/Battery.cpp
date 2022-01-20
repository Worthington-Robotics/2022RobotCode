#include "robot_lib/Battery.h"
#include <iostream>

namespace robot
{
    //Construct a Battery
    Battery::Battery(){
        //panel = frc::PowerDistribution{};
    }
    //get amount of power used over time from the PD panel
    double Battery::GetPowerUsage() { 
        double power = panel.GetTotalPower(); //power from panel
        double currentTime = frc::Timer::GetFPGATimestamp().to<double>();
        double timeDiff = currentTime - previousTime; //amount of time passed since last call
        double amt = power * timeDiff; //output
        powerUsed = previous + amt; 
        previous = amt;
        previousTime = currentTime;
        return amt;
    }
} // namespace robot