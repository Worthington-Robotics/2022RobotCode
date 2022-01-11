#include "robot_lib/BatteryInfo.h"

namespace robot
{
    BatteryInfo::BatteryInfo(){

    }

    double BatteryInfo::GetPowerUsage() { //get amount of power used over time from the PD panel
        double power = panel.GetTotalPower(); //power in joules from panel
        double currentTime = frc::Timer::GetFPGATimestamp().to<double>();
        double timeDiff = currentTime - previousTime; //amount of time passed since last call
        previous = power * timeDiff;
        previousTime = currentTime;
        return power * timeDiff;
    }
    bool BatteryInfo::UpdatePower() { //updates approximate amount of remaining battery and returns whether that level is low
        batteryPower -= previous;
        return batteryPower <= maxPower * 0.15;
    }
    
} // namespace robot