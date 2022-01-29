#include "frc/PowerDistribution.h"
#include "frc/Timer.h"
#include "frc/DriverStation.h"
namespace robot
{
    /**
     * A struct for encoding the PIDF gains of a control loop
     * @param p the proportional response to error (ex. kp * error / 1023 = motor responce)
     * @param i the integrated response of the error (ex. ki * S error dt / 1023 = motor responce) [This is VERY DANGEROUS and should be used with an iAccum]
     * @param d the derivative of error (ex. kd * [d/dt] error / 1023 = motor responce)
     * @param f the feedforward response to the setpoint (ex. kf * setpoint / 1023 = motor responce)
     */
    struct PIDFDiscriptor
    {
        double p;
        double i;
        double d;
        double f;
    };
    class PIDF {

        

    public:
        PIDF(PIDFDiscriptor);
        void setPIDFDisc(PIDFDiscriptor);
        void setInputRange(double inputRange);
        void setContinuous(bool continuous);
        void setSetpoint(double setpoint);
        void setIMax(double nIMax);
        double update(double reading, double now);
    private:
        double getContinuousError(double error);
        double getI(double error);
        double getD(double error);
        double iMax = 0.0;
        double inputRange = 0.0;
        double setpoint = 0.0;
        double previous = 0.0; //result of the last GetPowerUsage() call
        double errorSum = 0.0;
        double previousTime = 0.0; //timestamp of the last GetPowerUsage() call
        double dt = 0.0;
        bool continuous = false;
        PIDFDiscriptor mParams = {0, 0, 0, 0};
};
} // namespace robot