#pragma once

#include <ctre/Phoenix.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include "robot_lib/util/PIDF.h"

#define _USE_MATH_DEFINES
#include <cmath>

namespace robot
{


    struct rotationalData{
        double angleTicks;
        double speed;
    };

    /**
     * A struct for encoding the sensor data of a give swerve module, including information for the angle, drive, and CANcoder sections
     * @param angleRel the relative ticks of the given motor, should boot to zero if inline after being adjusted by startup offsets
     * @param drivePos the position of the drive motor in ticks, NOT the position of the robot
     * @param driveVel the velocity of the drive motor in ticks / 100ms NOT the speed of the robot
     * @param encAbs the absolute position of the CANcoder, and the refrence point for angleRel
     * @param angleCurrent provides the power of the motor controlling the angle of the swerve module
     * @param driveCurrent provides the power of the motor controlling the drive speed for the swerve module
     */
    struct sSensorData
    {
        double angleRel;
        double drivePos;
        double driveVel;
        double driveGoal;
        double angleCurrent;
        double driveCurrent;
    };

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
        SModule(int driveID, int angleID, int encoderID, double offset, PIDFDiscriptor dValues, PIDFDiscriptor aValues);

        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/
        void reset();

        rotationalData setMotors(frc::SwerveModuleState);

        void setMotorVelocity(frc::SwerveModuleState);

        frc::SwerveModuleState getState();

        sSensorData getData();

        void setInvertDrive(bool);

        void updateDrivePID(PIDFDiscriptor);

    private:
        /**
         * Configure the associated motor controllers with their settings as specified in constants
         **/ 
        double setpoint = 0;

        void configMotors(double, PIDFDiscriptor, PIDFDiscriptor);

        void updateSensorData();

        frc::SwerveModuleState optimize(frc::SwerveModuleState, double);

        //IO devices
        std::shared_ptr<TalonFX> drive, angle;
        std::shared_ptr<CANCoder> encod;
        
    };

} // namespace robot