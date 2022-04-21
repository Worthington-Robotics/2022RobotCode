#pragma once

#include "subsystems/Subsystem.h"
#include "Constants.h"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <can_msgs/msg/motor_msg.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

/* Motor libs and such */

#include <ctre/Phoenix.h>
// #include <TimeOfFlight.h>
#include <frc/DoubleSolenoid.h>
#include <robot_lib/motor_abs/talon_brushless.hpp>
#include <robot_lib/motor_abs/talon_brushed.hpp>
#include <robot_lib/motor_abs/solenoidWrapper.hpp>

namespace robot {

    class ExternIO : public Subsystem {
    public:
        ExternIO();

        void createRosBindings(rclcpp::Node *node) override;

        void reset() override;

        void onStart() override;

        void onLoop(double currentTime) override;

        void updateSensorData() override;

        void publishData() override;

        void setTOF0(const std_msgs::msg::Float32);
        void setTOF1(const std_msgs::msg::Float32);

        void enableDebug(bool debug) override;

    private:
        /* Publishers of sensor data */

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hoodEncoderPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr colesStupidFuckingLimelightPub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr flywheelEncoderPub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr externalTOFDistanceSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr internalTOFDistanceSub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr upperHoodLimitSwitchPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lowerHoodLimitSwitchPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hoodLimitSwitchResetPub;

        /* Messages used in the publishers listed above */

        sensor_msgs::msg::JointState hoodEncoderPosition;
        sensor_msgs::msg::JointState flywheelEncoderVelocity;
        std_msgs::msg::Float32 internalTOFDistance;
        std_msgs::msg::Bool upperHoodLimitSwitch;
        std_msgs::msg::Bool lowerHoodLimitSwitch;

        /* Motors to be controlled by coprocessor code and other systems */

        std::vector<motors::TalonBrushless*> motorsFX {
            new motors::TalonBrushless(INTAKE_MOTOR_ID, "intake"),
            new motors::TalonBrushless(FLYWHEEL_MOTOR_ID, "flywheel"),
            new motors::TalonBrushless(CLIMBER_L_MOTOR_ID, "climber_l"),
            new motors::TalonBrushless(CLIMBER_R_MOTOR_ID, "climber_r"),
            new motors::TalonBrushless(DELIVERY_MOTOR_ID, "delivery")
        };
        std::vector<motors::MotorContainer> motorsFXC;      
        std::vector<motors::TalonBrushed*> motorsSRX {
            new motors::TalonBrushed(INDEXER_MOTOR_ID, "indexer"),
            new motors::TalonBrushed(HOOD_MOTOR_ID, "hood")
        };
        std::vector<motors::MotorContainer> motorsSRXC;      
        //std::shared_ptr<frc::TimeOfFlight> internalTOF, externalTOF;


        std::vector<solenoid::Solenoid*> solenoids {
            new solenoid::Solenoid(frc::PneumaticsModuleType::CTREPCM, CLIMBER_SOLENOID_R_MAIN_HIGH_ID, CLIMBER_SOLENOID_R_MAIN_LOW_ID, "climber_r_main"),
            new solenoid::Solenoid(frc::PneumaticsModuleType::CTREPCM, INTAKE_SOLENOID_HIGH_ID, INTAKE_SOLENOID_LOW_ID, "intake")
        };

        std::vector<solenoid::SolenoidContainer> solenoidsC;

        bool hoodReset = false;
    };
}