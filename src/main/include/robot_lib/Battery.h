#include "frc/PowerDistribution.h"
#include "frc/Timer.h"
#include "frc/DriverStation.h"
namespace robot
{
    class Battery {
    public:
        Battery();
        double GetPowerUsage();
        double maxPower = 18.0; //out of 18.8, 18, and 17, this is a general average    should probably be changed later
        double batteryPower = 18.0; //amount of power left in the battery
        double powerUsed = 0.0; //average (ish) power usage up until the current time
    private:
        double previous = 0.0; //result of the last GetPowerUsage() call
        double previousTime = 0.0; //timestamp of the last GetPowerUsage() call
        double previousPower = 18.0; //level of the battery on the last update
        //void LowWarning();
        //frc::PowerDistribution panel{0, frc::PowerDistribution::ModuleType::kCTRE};
};
} // namespace robot