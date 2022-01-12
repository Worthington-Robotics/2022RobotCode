#include "robot_lib/Battery.h"

namespace robot
{
    Battery::Battery(){
        panel = frc::PowerDistribution();
    }
    void Battery::UpdateBattery() { //updates information about the battery
        //previousPower = batteryPower;
        Battery::GetPowerUsage();
        //Battery::LowWarning();
    }
    double Battery::GetPowerUsage() { //get amount of power used over time from the PD panel
        double power = panel.GetTotalPower(); //power in joules from panel
        double currentTime = frc::Timer::GetFPGATimestamp().to<double>();
        double timeDiff = currentTime - previousTime; //amount of time passed since last call
        double amt = power * timeDiff; //output
        powerUsed = previous + amt; 
        previous = amt;
        previousTime = currentTime;
        return amt;
    }
    // void Battery::LowWarning() { //updates approximate amount of remaining battery and warns if that level is low
    //     batteryPower -= previous; //I am 99% sure that this is not how battery capacity works, someone please enlighten me
    //     if (batteryPower <= maxPower * 0.15 && previousPower > maxPower * 0.15) { //make sure that warning only triggers once
    //         //display warning at driverstation?
    //     }
    // }
    
} // namespace robot