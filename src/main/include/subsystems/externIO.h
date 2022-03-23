#pragma once

#include "subsystems/Subsystem.h"
#include "Constants.h"

// msgs used in this package
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <can_msgs/msg/motor_msg.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// motor libs and such
#include <ctre/Phoenix.h>
// #include <TimeOfFlight.h>
#include <frc/DoubleSolenoid.h>
#include <robot_lib/motor_abs/talon_brushless.hpp>
#include <robot_lib/motor_abs/talon_brushed.hpp>
#include <robot_lib/motor_abs/solenoidWrapper.hpp>



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

        void updateSensorData() override;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void publishData() override;

        void setTOF(const std_msgs::msg::Float32);

        void enableDebug(bool debug) override;

    private:
        // publishers of sensor data
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hoodEncoderPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr colesStupidFuckingLimelightPub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr flywheelEncoderPub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr externalTOFDistanceSub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr upperHoodLimitSwitchPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lowerHoodLimitSwitchPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hoodLimitSwitchResetPub;

        // msgs used in the publishers of the listed above
        sensor_msgs::msg::JointState hoodEncoderPosition;
        sensor_msgs::msg::JointState flywheelEncoderVelocity;
        std_msgs::msg::Float32 internalTOFDistance;
        std_msgs::msg::Bool upperHoodLimitSwitch;
        std_msgs::msg::Bool lowerHoodLimitSwitch;

        // motors to be controlled by copro code and other systems
        std::vector<motors::TalonBrushless *> motorsFX {
            new motors::TalonBrushless(HOOD_MOTOR_ID, "hood"),
            new motors::TalonBrushless(INTAKE_MOTOR_ID, "intake"),
            new motors::TalonBrushless(FLYWHEEL_MOTOR_ID, "flywheel"),
            new motors::TalonBrushless(CLIMBER_L_MOTOR_ID, "climber_l"),
            new motors::TalonBrushless(CLIMBER_R_MOTOR_ID, "climber_r"),
            new motors::TalonBrushless(DELIVERY_MOTOR_ID, "delivery")
            };
        std::vector<motors::MotorContainer> motorsFXC;      
        std::vector<motors::TalonBrushed *> motorsSRX {
            new motors::TalonBrushed(INDEXER_MOTOR_ID, "indexer")
        };
        std::vector<motors::MotorContainer> motorsSRXC;      
        //std::shared_ptr<frc::TimeOfFlight> internalTOF, externalTOF;


        std::vector<solenoid::Solenoid*> solenoids {
            new solenoid::Solenoid(frc::PneumaticsModuleType::REVPH, CLIMBER_SOLENOID_L_MAIN_HIGH_ID, CLIMBER_SOLENOID_L_MAIN_LOW_ID, "climber_l_main"),
            new solenoid::Solenoid(frc::PneumaticsModuleType::REVPH, CLIMBER_SOLENOID_L_PIN_HIGH_ID, CLIMBER_SOLENOID_L_PIN_LOW_ID, "climber_l_pin"),
            new solenoid::Solenoid(frc::PneumaticsModuleType::REVPH, CLIMBER_SOLENOID_C_MAIN_HIGH_ID, CLIMBER_SOLENOID_C_MAIN_LOW_ID, "climber_c_main"),
            new solenoid::Solenoid(frc::PneumaticsModuleType::REVPH, CLIMBER_SOLENOID_C_PIN_HIGH_ID, CLIMBER_SOLENOID_C_PIN_LOW_ID, "climber_c_pin"),
            new solenoid::Solenoid(frc::PneumaticsModuleType::REVPH, CLIMBER_SOLENOID_R_MAIN_HIGH_ID, CLIMBER_SOLENOID_R_MAIN_LOW_ID, "climber_r_main"),
            new solenoid::Solenoid(frc::PneumaticsModuleType::REVPH, CLIMBER_SOLENOID_R_PIN_HIGH_ID, CLIMBER_SOLENOID_R_PIN_LOW_ID, "climber_r_pin"),
            new solenoid::Solenoid(frc::PneumaticsModuleType::REVPH, INTAKE_SOLENOID_HIGH_ID, INTAKE_SOLENOID_LOW_ID, "intake")
        };

        std::vector<solenoid::SolenoidContainer> solenoidsC;

        bool hoodReset = false;

    };

}