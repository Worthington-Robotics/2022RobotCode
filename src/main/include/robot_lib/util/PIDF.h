#pragma once
#include <units/time.h>
#include <can_msgs/srv/set_pidf_gains.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace robot {
    /**
     * A struct for encoding the PIDF gains of a control loop
     * @param p the proportional response to error (ex. kp * error / 1023 = motor responce)
     * @param i the integrated response of the error (ex. ki * S error dt / 1023 = motor responce) [This is VERY DANGEROUS and should be used with an iAccum]
     * @param d the derivative of error (ex. kd * [d/dt] error / 1023 = motor responce)
     * @param f the feedforward response to the setpoint (ex. kf * setpoint / 1023 = motor responce)
     **/
    struct PIDFDiscriptor {
        double p;
        double i;
        double d;
        double f;
    };
    class PIDF {
//CLEANUP: explain what ANY of these do
    public:

        PIDF(PIDFDiscriptor, std::string name);

        void setPIDFDisc(PIDFDiscriptor);

        void setInputRange(double inputRange);

        void setContinuous(bool continuous);

        void setSetpoint(double setpoint, bool resetIAccum);

        void setIMax(double nIMax);

        double update(double reading);

        void createRosBindings(rclcpp::Node * nodeHandle);
        
        void setPIDGains(const can_msgs::srv::SetPIDFGains::Request::SharedPtr req, can_msgs::srv::SetPIDFGains::Response::SharedPtr resp);

        double setpoint = 0.0;
    private:

        double getContinuousError(double error);

        double getI(double error);

        double getD(double error);
        
        double error = 0.0;
        double power = 0.0;
        double iMax = 0.0;
        double inputRange = 0.0;
        double previousE = 0.0; /* Result of the last GetPowerUsage() call */
        double errorSum = 0.0;
        units::second_t previousTime; /* Timestamp of the last GetPowerUsage() call */
        double dt = 0.0;
        bool continuous = false;
        std::string name;
        PIDFDiscriptor mParams = {0, 0, 0, 0};
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr errorPub, effortPub;
        rclcpp::Service<can_msgs::srv::SetPIDFGains>::SharedPtr gainService;
    };
} // namespace robot