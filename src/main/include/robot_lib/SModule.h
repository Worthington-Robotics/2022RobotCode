#pragma once

#include <ctre/Phoenix.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include "robot_lib/util/PIDF.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <can_msgs/srv/set_pidf_gains.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

namespace robot
{
    /**
     * SModule is a class designed to house all information assosiated with a single swerve module on the robot
     * It contains two CTRE TalonFX motors, and one CTRE CANcoder working in tandem to excecute swerveStates
     * as defined by WPI's swerveState class 
     */
    class SModule
    {
        public:

        /** The constructor for a SModule object
         * @param driveID the ID for a single CTRE TalonFX motor controller, which controls the linear velocity of the module
         * @param angleID the ID for a single CTRE TalonFX motor controller, which controls the angular position of the module
         * @param encoderID
         * @param offset
         * @param dValues
         * @param aValues
         */
        SModule(int driveID, int angleID, int encoderID, std::string name, double offset, PIDFDiscriptor dValues, PIDFDiscriptor aValues);

        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/
        void reset();

        void setMotors(frc::SwerveModuleState);

        void setMotorVelocity(frc::SwerveModuleState);

        void createRosBindings(rclcpp::Node *);

        frc::SwerveModuleState getState();

        void publishModuleInfo();

        void setInvertDrive(bool);

        void updateDrivePID(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request>,
                         std::shared_ptr<can_msgs::srv::SetPIDFGains::Response>);
        
        void updateAnglePID(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request>,
                         std::shared_ptr<can_msgs::srv::SetPIDFGains::Response>);

    private:
        /**
         * Configure the associated motor controllers with their settings as specified in constants
         **/ 
        void configMotors(double, PIDFDiscriptor, PIDFDiscriptor);

        void updateSensorData();

        frc::SwerveModuleState optimize(frc::SwerveModuleState, double);

        //IO devices
        std::shared_ptr<TalonFX> drive, angle;
        std::shared_ptr<CANCoder> encod;

        // inital config valuess
        PIDFDiscriptor angleConfig, driveConfig;
        double magnetOffset;

        double desiredAngle, desiredVelocity;

        std::string name;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
        rclcpp::Service<can_msgs::srv::SetPIDFGains>::SharedPtr anglePDIF, drivePIDF;
        
    };

} // namespace robot