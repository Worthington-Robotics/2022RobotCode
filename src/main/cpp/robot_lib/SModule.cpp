#include "robot_lib/SModule.h"
#include "Constants.h"
#include "Util.h"

#include <units/math.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

using std::placeholders::_1;
using std::placeholders::_2;

namespace robot {

    SModule::SModule(int driveID, int angleID, int encodID, std::string moduleName, double offset, PIDFDiscriptor dValues, PIDFDiscriptor aValues) {
        name = moduleName;
        angle = std::make_shared<TalonFX>(angleID, "Default Name");
        drive = std::make_shared<TalonFX>(driveID, "Default Name");
        encod = std::make_shared<CANCoder>(encodID, "Default Name");

        angleConfig = aValues;
        driveConfig = dValues;
        magnetOffset = offset;

        reset();
    }

    void SModule::configMotors(double offset, PIDFDiscriptor dValues, PIDFDiscriptor aValues) {
        // to convert from ticks to radians (2 radians per revolution) / (2048 ticks per revolution) * (GEAR REDUCTION)
        // to convert from ticks / 100ms to rps
        double bootPos = 0;
        /* Configure front left drive falcon */
        //CLEANUP: make these one macro
#define SSFP(SFE, X, Y) drive->SetStatusFramePeriod(SFE, X, Y)
        drive->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        SSFP(StatusFrameEnhanced::Status_2_Feedback0, 8, 0);
        SSFP(StatusFrameEnhanced::Status_1_General, 20, 0);
        SSFP(StatusFrameEnhanced::Status_4_AinTempVbat, 20, 0);
        SSFP(StatusFrameEnhanced::Status_6_Misc, 20, 0);
        SSFP(StatusFrameEnhanced::Status_7_CommStatus, 20, 0);
        SSFP(StatusFrameEnhanced::Status_9_MotProfBuffer, 249, 0);
        SSFP(StatusFrameEnhanced::Status_10_MotionMagic, 247, 0);
        SSFP(StatusFrameEnhanced::Status_12_Feedback1, 255, 0);
        SSFP(StatusFrameEnhanced::Status_13_Base_PIDF0, 253, 0);
        SSFP(StatusFrameEnhanced::Status_14_Turn_PIDF1, 251, 0);
        SSFP(StatusFrameEnhanced::Status_15_FirmareApiStatus, 249, 0);
        SSFP(StatusFrameEnhanced::Status_17_Targets1, 247, 0);
        drive->SetSensorPhase(false);
        drive->SetInverted(false);
        drive->SetNeutralMode(NeutralMode::Brake);
        drive->SelectProfileSlot(0, 0);
        drive->Config_kF(0, dValues.f, 0);
        drive->Config_kP(0, dValues.p, 0);
        drive->Config_kI(0, dValues.i, 0);
        drive->Config_kD(0, dValues.d, 0);
        drive->Config_IntegralZone(0, 300, 0);
        drive->ConfigVoltageCompSaturation(11, 0);
        drive->EnableVoltageCompensation(true);
        drive->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 55, 55, 1));

        // Configure front left angle falcon
        encod->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 100, 0);
        encod->ConfigSensorDirection(false, 0);
        encod->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
        encod->ConfigMagnetOffset(offset, 50);
        bootPos = std::fmod((encod->GetAbsolutePosition() / 180.0 * M_PI + 2.0 * M_PI), (2 * M_PI));
        double angleOffset = bootPos / SWERVE_ANGLE_POS_TTR;

        // double angleOffset = encod->GetAbsolutePosition() / SWERVE_ANGLE_GEARING * TICKS_PER_DEGREE;
        std::cout << name << " calibrated to " << angleOffset << std::endl;
        // std::cout << angleOffset << std::endl;

#undef SSFP
#define SSFP(SFE, X, Y) angle->SetStatusFramePeriod(SFE, X, Y)
        angle->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        SSFP(StatusFrameEnhanced::Status_2_Feedback0, 8, 0);
        SSFP(StatusFrameEnhanced::Status_1_General, 20, 0);
        SSFP(StatusFrameEnhanced::Status_4_AinTempVbat, 20, 0);
        SSFP(StatusFrameEnhanced::Status_6_Misc, 20, 0);
        SSFP(StatusFrameEnhanced::Status_7_CommStatus, 20, 0);
        SSFP(StatusFrameEnhanced::Status_9_MotProfBuffer, 249, 0);
        SSFP(StatusFrameEnhanced::Status_10_MotionMagic, 247, 0);
        SSFP(StatusFrameEnhanced::Status_12_Feedback1, 255, 0);
        SSFP(StatusFrameEnhanced::Status_13_Base_PIDF0, 8, 0);
        SSFP(StatusFrameEnhanced::Status_14_Turn_PIDF1, 251, 0);
        SSFP(StatusFrameEnhanced::Status_15_FirmareApiStatus, 249, 0);
        SSFP(StatusFrameEnhanced::Status_17_Targets1, 247, 0);
#undef SSFP
        angle->SetSensorPhase(true);
        angle->SetInverted(true);
        angle->SetNeutralMode(NeutralMode::Brake);
        angle->SelectProfileSlot(0, 0);
        angle->Config_kF(0, aValues.f, 0);
        angle->Config_kP(0, aValues.p, 0);
        angle->Config_kI(0, aValues.i, 0);
        angle->Config_kD(0, aValues.d, 0);
        angle->SelectProfileSlot(0, 0);
        angle->Config_IntegralZone(0, 8000, 0);
        angle->ConfigVoltageCompSaturation(11, 0);
        angle->EnableVoltageCompensation(true);
        angle->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 55, 55, 2));
        angle->SetSelectedSensorPosition(-angleOffset, 0);
    }

    void SModule::createRosBindings(rclcpp::Node* nodeHandle) {
        jointStatePub = nodeHandle->create_publisher<sensor_msgs::msg::JointState>("/drive/" + name + "/joint_state", DEFAULT_QOS);

        anglePDIF = nodeHandle->create_service<can_msgs::srv::SetPIDFGains>("/drive/" + name + "/angle/pidfset", std::bind(&SModule::updateAnglePID, this, _1, _2));
        drivePIDF = nodeHandle->create_service<can_msgs::srv::SetPIDFGains>("/drive/" + name + "/drive/pidfset", std::bind(&SModule::updateDrivePID, this, _1, _2));
    }

    void SModule::setInvertDrive(bool invert) {
        drive->SetInverted(invert);
    }

    void SModule::updatePID(std::shared_ptr<TalonFX> motor, const can_msgs::srv::SetPIDFGains::Request::SharedPtr req, can_msgs::srv::SetPIDFGains::Response::SharedPtr resp) {
        motor->Config_kF(0, req->k_f, 0);
        motor->Config_kP(0, req->k_p, 0);
        motor->Config_kI(0, req->k_i, 0);
        motor->Config_kD(0, req->k_d, 0);

        resp->success = true;
    }
    
    void SModule::updateDrivePID(const can_msgs::srv::SetPIDFGains::Request::SharedPtr req, can_msgs::srv::SetPIDFGains::Response::SharedPtr resp) {
        updatePID(drive, req, resp);
    }

    void SModule::updateAnglePID(const can_msgs::srv::SetPIDFGains::Request::SharedPtr req, can_msgs::srv::SetPIDFGains::Response::SharedPtr resp) {
        updatePID(angle, req, resp);
    }

    void SModule::reset() {
        configMotors(magnetOffset, driveConfig, angleConfig);
    }

    void SModule::setMotorVelocity(frc::SwerveModuleState ss) {
        double currentTicks = angle->GetSelectedSensorPosition();
        auto ssO = frc::SwerveModuleState::Optimize(ss, units::degree_t(currentTicks / TICKS_PER_DEGREE * SWERVE_ANGLE_GEARING));
        // auto ssO = optimizeCTREModule(ss, units::degree_t(currentTicks / TICKS_PER_DEGREE / (64 / 5)));

        desiredVelocity = ssO.speed.to<double>() * (2048 * 39.37 * 6.12) / (4 * M_PI * 10);
        desiredAngle = ssO.angle.Degrees().to<double>() * TICKS_PER_DEGREE / SWERVE_ANGLE_GEARING;

        angle->Set(ControlMode::Position, desiredAngle);
        drive->Set(ControlMode::Velocity, desiredVelocity);
    }

    void SModule::setMotors(frc::SwerveModuleState ss) {
        double currentTicks = angle->GetSelectedSensorPosition();
        // auto ssO = frc::SwerveModuleState::Optimize(ss, units::degree_t(currentTicks / TICKS_PER_DEGREE / (64 / 5)));
        auto ssO = optimizeCTREModule(ss, units::degree_t(currentTicks / TICKS_PER_DEGREE * SWERVE_ANGLE_GEARING));

        desiredVelocity = ssO.speed.to<double>();
        desiredAngle = ssO.angle.Degrees().to<double>() * TICKS_PER_DEGREE / SWERVE_ANGLE_GEARING;

        angle->Set(ControlMode::Position, desiredAngle);
        drive->Set(ControlMode::PercentOutput, desiredVelocity);
    }

    frc::SwerveModuleState SModule::getState() {
        //(ticks / 100ms) / (ticks / revolution) * (Circumfrence [Diameter * Pi] / revolution) * (1s / 100ms) / (1m / 39.37in)
        return frc::SwerveModuleState{units::meters_per_second_t{(drive->GetSelectedSensorVelocity() / (2048 * 39.37 * 6.12)) * (4 * M_PI * 10)},
                                      frc::Rotation2d{units::degree_t{angle->GetSelectedSensorPosition() / TICKS_PER_DEGREE / (64 / 5)}}};
    }

    void SModule::publishModuleInfo() {
        auto msg = sensor_msgs::msg::JointState();

        frc::SmartDashboard::PutNumber("swerve_module/" + name + "/module_angle", std::fmod(angle->GetSelectedSensorPosition() / TICKS_PER_DEGREE / (64 / 5) + 360, 360));
        frc::SmartDashboard::PutNumber("swerve_module/" + name + "/module_velocity", (drive->GetSelectedSensorVelocity() / (2048 * 39.37 * 6.12)) * (4 * M_PI * 10));

        /* Push in the names */
        msg.name = {"angle", "angle_goal", "drive", "drive_goal"};

        /* Push in the positions */
        msg.position = {
            angle->GetSelectedSensorPosition(),
            desiredAngle,
            drive->GetSelectedSensorPosition(),
            0.0
        };

        /* Push in the velocities */
        msg.velocity = {
            angle->GetSelectedSensorVelocity(),
            0.0,
            drive->GetSelectedSensorVelocity(),
            desiredVelocity
        };

        /* Push in the efforts */
        msg.effort = {
            std::abs(angle->GetStatorCurrent()),
            0.0,
            std::abs(drive->GetStatorCurrent()),
            0.0
        };

        jointStatePub->publish(msg);
    }

    frc::SwerveModuleState SModule::optimizeCTREModule(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle) {
        double targetAngle = correctScope(
            currentAngle.Degrees().to<double>(),
            desiredState.angle.Degrees().to<double>()
        );
        double targetSpeed = desiredState.speed.to<double>();
        double delta = targetAngle - currentAngle.Degrees().to<double>();
        if (std::fabs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }        
        return frc::SwerveModuleState{units::meters_per_second_t{targetSpeed}, frc::Rotation2d{units::degree_t{targetAngle}}};
    }

    //CLEANUP: what the hell is this 
    double SModule::correctScope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = std::fmod(scopeReference, 360);
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        //CLEANUP: you could probably use modulo for this
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
    
} // namespace robot
