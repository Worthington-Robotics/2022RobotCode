#pragma once

#include "subsystems/Subsystem.h"
#include "Constants.h"
#include "Util.h"

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

        void setTOF0(const MSG_FLOAT);
        void setTOF1(const MSG_FLOAT);

        void enableDebug(bool debug) override;

    private:
        /* Publishers of sensor data */

        ROS_PUB(sensor_msgs::msg::JointState) hoodEncoderPub;
        ROS_PUB(MSG_BOOL) colesStupidFuckingLimelightPub;
        ROS_PUB(sensor_msgs::msg::JointState) flywheelEncoderPub;
        ROS_SUB(MSG_FLOAT) externalTOFDistanceSub;
        ROS_SUB(MSG_FLOAT) internalTOFDistanceSub;
        ROS_PUB(MSG_BOOL) upperHoodLimitSwitchPub;
        ROS_PUB(MSG_BOOL) lowerHoodLimitSwitchPub;
        ROS_PUB(MSG_BOOL) hoodLimitSwitchResetPub;

        /* Messages used in the publishers listed above */

        sensor_msgs::msg::JointState hoodEncoderPosition;
        sensor_msgs::msg::JointState flywheelEncoderVelocity;
        MSG_FLOAT internalTOFDistance;
        MSG_BOOL upperHoodLimitSwitch;
        MSG_BOOL lowerHoodLimitSwitch;

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

} // namespace robot