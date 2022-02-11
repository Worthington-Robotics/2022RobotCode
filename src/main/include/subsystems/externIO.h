#pragma once

#include "subsystems/Subsystem.h"
#include "Constants.h"

// msgs used in this package
#include <std_msgs/msg/Float32.hpp>
#include <std_msgs/msg/Int16.hpp>
#include <std_msgs/msg/Bool.hpp>

// motor libs and such
#include <ctre/Phoenix.h>
#include <TimeOfFlight.h>



namespace robot
{

    class ExternIO : public Subsystem
    {
    public:
        ExternIO();

        /**
         * Override this function in order to create pulbishers or subscribers against the parent node.
         * NOTE: This function is automatically called by the subsystem manager on registration
         **/
        void createRosBindings(rclcpp::Node *node) override;

        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/
        void reset() override;

        /**
         * Overrride this function with any code needed to be called only once on the first onloop iteration
         **/
        void onStart() override;

        /**
         * Override this function for any code that must be called periodically by the subsystem
         **/
        void onLoop(double currentTime) override;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void publishData() override;

        void enableDebug(bool debug) override;

    private:
        // publishers of sensor data
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hoodEncoderPositionPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flywheelEncoderVelocityPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr externalTOFDistancePub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr internalTOFDistancePub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr upperHoodLimitSwitchPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lowerHoodLimitSwitchPub;

        // subscribers of motor demands
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hoodMotorPositionDemandSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr flywheelMotorVelocityDemandSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr deliveryMotorDemandSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr intakeMotorDemandSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr climberMotorLDemandSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr climberMotorCDemandSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr climberMotorRDemandSub;

        // msgs used in the publishers of the listed above
        std_msgs::msg::Float32 hoodEncoderPosition;
        std_msgs::msg::Float32 flywheelEncoderVelocity;
        std_msgs::msg::Bool externalTOFDistance;
        std_msgs::msg::Bool internalTOFDistance;
        std_msgs::msg::Bool upperHoodLimitSwitch;
        std_msgs::msg::Bool lowerHoodLimitSwitch;

        // cached data values from the subscribers
        double hoodMotorPositionDemand = 0;
        double flywheelMotorVelocityDemand = 0;
        double deliveryMotorDemand = 0;
        double intakeMotorDemand = 0;
        double climberLMotorDemand = 0;
        double climberCMotorDemand = 0;
        double climberRMotorDemand = 0;

        //subscriber callback functions
        
        void setHoodMotorPositionDemand(const std_msgs::msg::Float32);
        void setFlywheelMotorVelocityDemand(const std_msgs::msg::Float32);
        void setDeliveryMotorDemand(const std_msgs::msg::Float32);
        void setIntakeMotorDemand(const std_msgs::msg::Float32);
        void setClimberLMotorDemand(const std_msgs::msg::Float32);
        void setClimberCMotorDemand(const std_msgs::msg::Float32);
        void setClimberRMotorDemand(const std_msgs::msg::Float32);

        // motors to be controlled by copro code and other systems
        std::shared_ptr<TalonFX> flywheelMotor, deliveryMotor, intakeMotor, climberMotorL, climberMotorR, climberMotorC;
        std::shared_ptr<TalonSRX> hoodMotor;
        std::shared_ptr<frc::TimeOfFlight> internalTOF, externalTOF;
    };
}