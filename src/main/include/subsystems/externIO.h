#pragma once

#include "subsystems/Subsystem.h"
#include "Constants.h"

// msgs used in this package
#include <std_msgs/msg/Float32.hpp>
#include <std_msgs/msg/Int16.hpp>
#include <std_msgs/msg/Bool.hpp>
#include <can_msgs/msg/motor_msg.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// motor libs and such
#include <ctre/Phoenix.h>
//#include <TimeOfFlight.h>
#include <frc/DoubleSolenoid.h>
#include <robot_lib/motor_abs/talon_brushless.hpp>
#include <robot_lib/motor_abs/talon_brushed.hpp>



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
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hoodEncoderPub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr flywheelEncoderPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr externalTOFDistancePub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr internalTOFDistancePub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr upperHoodLimitSwitchPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lowerHoodLimitSwitchPub;

        // subscribers of motor demands
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr climberSolenoidLRStateSub;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr climberSolenoidCStateSub;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr intakeSolenoidStateSub;

        // msgs used in the publishers of the listed above
        sensor_msgs::msg::JointState hoodEncoderPosition;
        sensor_msgs::msg::JointState flywheelEncoderVelocity;
        std_msgs::msg::Float32 externalTOFDistance;
        std_msgs::msg::Float32 internalTOFDistance;
        std_msgs::msg::Bool upperHoodLimitSwitch;
        std_msgs::msg::Bool lowerHoodLimitSwitch;

        // cached data values from the subscribers
        int climberSolenoidLRState = 0;
        int climberSolenoidCState = 0;
        int intakeSolenoidState = 0;

        //subscriber callback functions
        void setClimberSolenoidLRState(const std_msgs::msg::Int16);
        void setClimberSolenoidCState(const std_msgs::msg::Int16); 
        void setIntakeSolenoidState(const std_msgs::msg::Int16);

        // motors to be controlled by copro code and other systems
        std::vector<std::shared_ptr<motors::TalonBrushless>> motorsFX {
            std::make_shared<motors::TalonBrushless>(INTAKE_MOTOR_ID, "intake_motor"),
            std::make_shared<motors::TalonBrushless>(INDEXER_MOTOR_ID, "indexer_motor"),
            std::make_shared<motors::TalonBrushless>(DELIVERY_MOTOR_ID, "delivery_motor"),
            std::make_shared<motors::TalonBrushless>(FLYWHEEL_MOTOR_ID, "flywheel_motor"),
            std::make_shared<motors::TalonBrushless>(CLIMBER_L_MOTOR_ID, "climber_l_motor"),
            std::make_shared<motors::TalonBrushless>(CLIMBER_C_MOTOR_ID, "climber_c_motor"),
            std::make_shared<motors::TalonBrushless>(CLIMBER_R_MOTOR_ID, "climber_r_motor")
            };
        std::vector<motors::MotorContainer> motorsFXC;      
        std::vector<std::shared_ptr<motors::TalonBrushed>> motorsSRX {
            std::make_shared<motors::TalonBrushed>(HOOD_MOTOR_ID, "hood_motor")
        };
        std::vector<motors::MotorContainer> motorsSRXC;      
        //std::shared_ptr<frc::TimeOfFlight> internalTOF, externalTOF;
        std::shared_ptr<frc::DoubleSolenoid> climberSolenoidLR, climberSolenoidC, intakeSolenoid;
    };
}