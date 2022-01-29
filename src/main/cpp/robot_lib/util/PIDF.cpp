#include "robot_lib/util/PIDF.h"

namespace robot
{

        //constructor!!
        PIDF::PIDF(PIDFDiscriptor disc)
        {
            mParams = disc;
        }

        //getters and setters, pretty self explainitory
        void PIDF::setPIDFDisc(PIDFDiscriptor disc)
        {
            mParams = disc;
        }

        void PIDF::setInputRange(double nInputRange)
        {
            inputRange = nInputRange;
        }

        void PIDF::setContinuous(bool nContinuous)
        {
            continuous = nContinuous;
        }

        void PIDF::setSetpoint(double nSetpoint)
        {
            setpoint = nSetpoint;
        }

        /**
         * updates the PIDF loop with a new value and time
         * @param reading the raw value of whatever you are reading in, might be adjusted by continuous
         * @param now the timestamp now, like *right now*
         * @return the PIDF output
         */
        double PIDF::update(double reading, double now)
        {
            double error = getContinuousError(reading);
            double power = (mParams.f * setpoint + mParams.p * error + mParams.i * getI(error) + mParams.d * getD(reading));
            dt = now - previousTime;
            previousTime = now;
            previous = reading;
            return power;
        }

        double PIDF::getI(double error)
        {
            errorSum += (error * dt);
            double result = errorSum;
            //if iMax has been set, actually apply it
            if(iMax != 0){
                //get the sign of the integral of the error
                int sign = (int)(errorSum / std::abs(errorSum));
                //bound the result
                result = sign * std::min(std::abs(errorSum), iMax);
            }
            return  result;
        }

        void PIDF::setIMax(double nIMax)
        {
            iMax = std::abs(nIMax);
        }

        double PIDF::getD(double reading)
        {
            return ((reading - previous) / dt);
        }

    /**
     * Wraps error around for continuous inputs. The original
     * error is returned if continuous mode is disabled.
     *
     * @param error The current error of the PID controller.
     * @return Error for continuous inputs.
     */
    double PIDF::getContinuousError(double error) {
        if (continuous && inputRange > 0) {
            error = std::fmod(error, inputRange);
            if (std::abs(error) > inputRange / 2) {
                if (error > 0) {
                    return error - inputRange;
                } else {
                    return error + inputRange;
                }
            }
        }
        return error;
    }
} // namespace robot