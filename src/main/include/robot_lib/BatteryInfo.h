#include "frc/PowerDistribution.h"
#include "frc/Timer.h"
namespace robot
{
    class BatteryInfo {
    public:
        BatteryInfo();
        double GetPowerUsage();
        bool UpdatePower();
        double maxPower = 18.0;
        double batteryPower = 18.0;
        
    private:
        double previous = 0.0;
        double previousTime = 0.0;
        frc::PowerDistribution panel;
};
} // namespace robot