#include "robot_lib/SModule.h"
#include "Constants.h"
#include <units/math.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using std::placeholders::_1;
using std::placeholders::_2;

namespace robot
{

    SModule::SModule(int driveID, int angleID, int encodID, std::string moduleName, double offset, PIDFDiscriptor dValues, PIDFDiscriptor aValues)
    {
        name = moduleName;
        angle = std::make_shared<TalonFX>(angleID, "Default Name");
        drive = std::make_shared<TalonFX>(driveID, "Default Name");
        encod = std::make_shared<CANCoder>(encodID, "Default Name");
        
        angleConfig = aValues;
        driveConfig = dValues;
        magnetOffset = offset;

        reset();
    }

    void SModule::configMotors(double offset, PIDFDiscriptor dValues, PIDFDiscriptor aValues)
    {
        //to convert from ticks to radians (2 radians per revolution) / (2048 ticks per revolution) * (GEAR REDUCTION)
        //to convert from ticks / 100ms to rps
        double bootPos = 0;
        // Configure front left drive falcon
        drive->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 8, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 20, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 20, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 20, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 249, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 247, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 255, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 253, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 251, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 249, 0);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 247, 0);
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
        drive->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 70, 1));

        // Configure front left angle falcon
        encod->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 100, 0);
        encod->ConfigSensorDirection(false, 0);
        encod->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
        encod->ConfigMagnetOffset(offset);
        bootPos = std::fmod((encod->GetAbsolutePosition() / 180.0 * M_PI + 2.0 * M_PI), (2 * M_PI));
        //std::cout << bootPos << std::endl;
        double angleOffset = bootPos / SWERVE_ANGLE_POS_TTR;
        //std::cout << angleOffset << std::endl;

        angle->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 8, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 20, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 20, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 20, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 249, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 247, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 255, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 8, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 251, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 249, 0);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 247, 0);
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
        angle->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 70, 1));
        angle->SetSelectedSensorPosition(-angleOffset, 0);
    }

    void SModule::createRosBindings(rclcpp::Node * nodeHandle){
        jointStatePub = nodeHandle->create_publisher<sensor_msgs::msg::JointState>("/drive/" + name + "/joint_state", rclcpp::SystemDefaultsQoS());

        anglePDIF = nodeHandle->create_service<can_msgs::srv::SetPIDFGains>("/drive/" + name + "/angle/pidfset", std::bind(&SModule::updateAnglePID, this, _1, _2));
        drivePIDF = nodeHandle->create_service<can_msgs::srv::SetPIDFGains>("/drive/" + name + "/drive/pidfset", std::bind(&SModule::updateDrivePID, this, _1, _2));
    }

    void SModule::setInvertDrive(bool invert){
        drive->SetInverted(invert);
    }

    void SModule::updateDrivePID(const can_msgs::srv::SetPIDFGains::Request::SharedPtr req, can_msgs::srv::SetPIDFGains::Response::SharedPtr resp){
        drive->Config_kF(0, req->k_f, 0);
        drive->Config_kP(0, req->k_p, 0);
        drive->Config_kI(0, req->k_i, 0);
        drive->Config_kD(0, req->k_d, 0);

        resp->success = true;
    }
    
    void SModule::updateAnglePID(const can_msgs::srv::SetPIDFGains::Request::SharedPtr req, can_msgs::srv::SetPIDFGains::Response::SharedPtr resp){
        angle->Config_kF(0, req->k_f, 0);
        angle->Config_kP(0, req->k_p, 0);
        angle->Config_kI(0, req->k_i, 0);
        angle->Config_kD(0, req->k_d, 0);

        resp->success = true;
    }

    void SModule::reset()
    {
        configMotors(magnetOffset, driveConfig, angleConfig);
    }

    void SModule::setMotorVelocity(frc::SwerveModuleState ss)
    {
        double currentTicks = angle->GetSelectedSensorPosition();
        auto ssO = frc::SwerveModuleState::Optimize(ss, units::degree_t(currentTicks / TICKS_PER_DEGREE / (64 / 5)));

        desiredVelocity =  ssO.speed.to<double>() * (2048 * 39.37 * 6.12) / (4 * M_PI * 10);
        desiredAngle = ssO.angle.Degrees().to<double>() * TICKS_PER_DEGREE * (64 / 5);

        angle->Set(ControlMode::Position, desiredAngle);
        drive->Set(ControlMode::Velocity, desiredVelocity);
    }

    void SModule::setMotors(frc::SwerveModuleState ss)
    {      
        double currentTicks = angle->GetSelectedSensorPosition();
        auto ssO = frc::SwerveModuleState::Optimize(ss, units::degree_t(currentTicks / TICKS_PER_DEGREE / (64 / 5)));

        desiredVelocity =  ssO.speed.to<double>();
        desiredAngle = ssO.angle.Degrees().to<double>() * TICKS_PER_DEGREE * (64 / 5);

        // int rotations = (int)(currentTicks / (TICKS_PER_DEGREE * 360) / (64 / 5));
        // double targetPoint = rotations * (TICKS_PER_DEGREE * 360) / (64 / 5);

        // double targetPointOne = targetPoint + ssO.angle.Degrees().to<double>() * TICKS_PER_DEGREE * (64/5);
        // double differenceOne = abs(targetPointOne - currentTicks);

        // double targetPointTwo = targetPointOne - (TICKS_PER_DEGREE * 360) / (64 / 5) ; 
        // double differenceTwo = abs(targetPointTwo - currentTicks);

        // double targetPointThree = targetPointTwo + 2 * (TICKS_PER_DEGREE * 360) / (64 / 5);
        // double differenceThree = abs(targetPointThree - currentTicks);


        // desiredVelocity = ssO.speed.to<double>();
        // if(differenceOne < differenceTwo && differenceOne < differenceThree){
        //     desiredAngle = targetPointOne;
        // } else if (differenceTwo < differenceOne && differenceTwo < differenceThree){
        //     desiredAngle = targetPointTwo;
        // } else {
        //     desiredAngle = targetPointThree;
        // }

        angle->Set(ControlMode::Position, desiredAngle);
        drive->Set(ControlMode::PercentOutput, desiredVelocity);       
    }

    frc::SwerveModuleState SModule::getState()
    {
        //(ticks / 100ms) / (ticks / revolution) * (Circumfrence [Diameter * Pi] / revolution) * (1s / 100ms) / (1m / 39.37in)
        return frc::SwerveModuleState{units::meters_per_second_t{(drive->GetSelectedSensorVelocity() / (2048 * 39.37 * 6.12)) * (4 * M_PI * 10)},
         frc::Rotation2d{units::degree_t{angle->GetSelectedSensorPosition() / TICKS_PER_DEGREE / (64 / 5)}}};
    }
 
    void SModule::publishModuleInfo(){
        auto msg = sensor_msgs::msg::JointState();

        frc::SmartDashboard::PutNumber("swerve_module/" + name + "/module_angle", std::fmod(angle->GetSelectedSensorPosition() / TICKS_PER_DEGREE / (64 / 5) + 360, 360));
        frc::SmartDashboard::PutNumber("swerve_module/" + name + "/module_velocity", (drive->GetSelectedSensorVelocity() / (2048 * 39.37 * 6.12)) * (4 * M_PI * 10));

        // push in the names
        msg.name = {"angle", "angle_goal", "drive", "drive_goal"};

        // push in the positions
        msg.position = {
            angle->GetSelectedSensorPosition(),
            desiredAngle,
            drive->GetSelectedSensorPosition(),
            0.0
        };

        // push in the velocities
        msg.velocity = {
            angle->GetSelectedSensorVelocity(),
            0.0,
            drive->GetSelectedSensorVelocity(),
            desiredVelocity
        };

        // push in the efforts
        msg.effort = {
            std::abs(angle->GetStatorCurrent()), 
            0.0, 
            std::abs(drive->GetStatorCurrent()), 
            0.0
        };

        jointStatePub->publish(msg);
    }

} // namespace robot
