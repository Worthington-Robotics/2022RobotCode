#include "robot_lib/util/PIDF.h"
#include <frc/Timer.h>
#include <cmath>
#include <iostream>
using std::placeholders::_1;
using std::placeholders::_2;

namespace robot
{

    // constructor!!
    PIDF::PIDF(PIDFDiscriptor disc, std::string rosName)
    {
        mParams = disc;
        name = rosName;
    }

    // getters and setters, pretty self explainitory
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

    void PIDF::setSetpoint(double nSetpoint, bool resetID)
    {
        setpoint = nSetpoint;
        if (resetID)
        {
            previousTime = frc::Timer::GetFPGATimestamp();
            errorSum = 0;
            dt = 0;
        }
    }

    /**
     * updates the PIDF loop with a new value and time
     * @param reading the raw value of whatever you are reading in, might be adjusted by continuous
     * @param now the timestamp now, like *right now*
     * @return the PIDF output
     */
    double PIDF::update(double reading)
    {
        units::second_t now = frc::Timer::GetFPGATimestamp();
        error = getContinuousError(reading - setpoint);
        dt = now.to<double>() - previousTime.to<double>();
        power = (mParams.f * setpoint + mParams.p * error + mParams.i * getI(error) + mParams.d * getD(error));
        previousTime = now;
        previousE = error;

        std_msgs::msg::Float32 errorMsg, powerMsg;

        errorMsg.data = error;
        powerMsg.data = power;

        errorPub->publish(errorMsg);
        effortPub->publish(powerMsg);
        return power;
    }

    double PIDF::getI(double error)
    {
        errorSum += (error * dt);
        // if iMax has been set, actually apply it
        if (iMax != 0)
        {
            // get the sign of the integral of the error
            int sign = (int)(errorSum / std::fabs(errorSum));
            // bound the result
            errorSum = sign * std::min(std::fabs(errorSum), iMax);
        }
        return errorSum;
    }

    void PIDF::setIMax(double nIMax)
    {
        iMax = std::abs(nIMax);
    }

    double PIDF::getD(double error)
    {
        if (dt == 0)
        {
            return 0;
        }
        return ((error - previousE) / dt);
    }

    /**
     * Wraps error around for continuous inputs. The original
     * error is returned if continuous mode is disabled.
     *
     * @param error The current error of the PID controller.
     * @return Error for continuous inputs.
     */
    double PIDF::getContinuousError(double error)
    {
        if (continuous && inputRange > 0)
        {
            error = std::fmod(error, inputRange);
            if (std::abs(error) > inputRange / 2)
            {
                if (error > 0)
                {
                    return error - inputRange;
                }
                else
                {
                    return error + inputRange;
                }
            }
        }
        return error;
    }

    void PIDF::setPIDGains(const can_msgs::srv::SetPIDFGains::Request::SharedPtr req, can_msgs::srv::SetPIDFGains::Response::SharedPtr resp)
    {
        mParams.f = req->k_f;
        mParams.p = req->k_p;
        mParams.i = req->k_i;
        mParams.d = req->k_d;
        iMax = req->i_max;

        resp->success = true;
    }

    void PIDF::createRosBindings(rclcpp::Node *nodeHandle)
    {
        // error, effort, setpoint
        errorPub = nodeHandle->create_publisher<std_msgs::msg::Float32>("/pidf/" + name + "/error", rclcpp::SystemDefaultsQoS());
        effortPub = nodeHandle->create_publisher<std_msgs::msg::Float32>("/pidf/" + name + "/effort", rclcpp::SystemDefaultsQoS());

        gainService = nodeHandle->create_service<can_msgs::srv::SetPIDFGains>("/pidf/" + name + "/pidfset", std::bind(&PIDF::setPIDGains, this, _1, _2));
    }
} // namespace robot