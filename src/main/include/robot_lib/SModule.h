#pragma once

#include <ctre/Phoenix.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>

#define _USE_MATH_DEFINES
#include <cmath>

namespace robot
{

    /**
     * A struct for encoding the PIDF gains of a control loop
     * @param p the proportional response to error (ex. kp * error / 1023 = motor responce)
     * @param i the integrated response of the error (ex. ki * S error dt / 1023 = motor responce) [This is VERY DANGEROUS and should be used with an iAccum]
     * @param d the derivative of error (ex. kd * [d/dt] error / 1023 = motor responce)
     * @param f the feedforward response to the setpoint (ex. kf * setpoint / 1023 = motor responce)
     */
    struct PIDF
    {
        double p;
        double i;
        double d;
        double f;
    };

    /**
     * A struct for encoding the sensor data of a give swerve module, including information for the angle, drive, and CANcoder sections
     * @param angleRel the relative ticks of the given motor, should boot to zero if inline after being adjusted by startup offsets
     * @param drivePos the position of the drive motor in ticks, NOT the position of the robot
     * @param driveVel the velocity of the drive motor in ticks / 100ms NOT the speed of the robot
     * @param encAbs the absolute position of the CANcoder, and the refrence point for angleRel
     */
    struct sSensorData
    {
        double angleRel;
        double drivePos;
        double driveVel;
        double encAbs;
        double currentOne;
        double currentTwo;
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
        SModule(int driveID, int angleID, int encoderID, double offset, PIDF dValues, PIDF aValues);

        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/
        void reset();

        void setMotors(frc::SwerveModuleState);

        frc::SwerveModuleState getState();

        sSensorData getData();

        void setInvertDrive(bool);

    private:
        /**
         * Configure the associated motor controllers with their settings as specified in constants
         **/ 
        void configMotors(double, PIDF, PIDF);

        void updateSensorData();

        frc::SwerveModuleState optimize(frc::SwerveModuleState, double);

        //IO devices
        std::shared_ptr<TalonFX> drive, angle;
        std::shared_ptr<CANCoder> encod;
        
    };

} // namespace robot

