#pragma once

#include "subsystems/Subsystem.h"
#include "robot_lib/PurePursuitController.h"
#include "robot_lib/SModule.h"
#include "rospathmsgs/srv/get_path.hpp"
#include "Constants.h"
#include "Util.h"
#include "robot_lib/PurePursuitController.h"
#include "robot_lib/util/PIDF.h"

#include <ctre/Phoenix.h>
#include <frc/controller/RamseteController.h>
#include <rclcpp/rclcpp.hpp>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <can_msgs/srv/set_pidf_gains.hpp>
#include <can_msgs/msg/motor_msg.hpp>
#include <std_msgs/msg/int16.hpp>
#include <rospathmsgs/srv/get_path.hpp>
#include <std_msgs/msg/float32.hpp>
#include "autobt_msgs/srv/string_service.hpp"

#define DEBUG_enable
#include <std_msgs/msg/bool.hpp>

#define CREATE_SMODULE(CODE, NAME) std::make_shared<SModule>(DRIVE_CODE_DRIVE, DRIVE_CODE_ANGLE, DRIVE_CODE_ENCOD, NAME, CODE_ABS_OFFSET, PIDFDiscriptor{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF}, PIDFDiscriptor{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF})
#define SPEED_STOPPED frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}

namespace robot {

    /* Possible control states for the drivetrain to be in */
    enum ControlState {
        kOPEN_LOOP_ROBOT_REL,
        kOPEN_LOOP_FIELD_REL,
        kVELOCITY_TWIST,
        kPURSUIT
    };

    class Drivetrain : public Subsystem {
    public:
    
        Drivetrain();

        void createRosBindings(rclcpp::Node *node) override;

        void reset() override;

        void onStart() override;

        void onLoop(double currentTime) override;

        void publishData() override;

        void enableDebug(bool debug) override;

        /* Request a path from the path generator with the given name, assuming it is baked */
        bool enablePathFollower(std::string name);
        
        void setHeadingControlGains(PIDFDiscriptor);

        void enableOpenLoop();

        void resetPose();

    private:
        bool DEBUG = true;

        /* IO devices */

        std::shared_ptr<SModule> frontRMod, frontLMod, rearRMod, rearLMod;
        std::vector<std::shared_ptr<SModule>> sModules;
        std::shared_ptr<PigeonIMU> imu;

        /* Underlying controllers */

        frc::Translation2d sFrontRight{CHASSIS_LENGTH, CHASSIS_LENGTH};
        frc::Translation2d sFrontLeft{CHASSIS_LENGTH, -CHASSIS_LENGTH};
        frc::Translation2d sRearRight{-CHASSIS_LENGTH, CHASSIS_LENGTH};
        frc::Translation2d sRearLeft{-CHASSIS_LENGTH, -CHASSIS_LENGTH};
        frc::SwerveDriveKinematics<4> sKinematics{sFrontRight, sFrontLeft, sRearRight, sRearLeft};
        frc::SwerveDriveOdometry<4> sOdom{sKinematics, frc::Rotation2d{units::degree_t{0}}};
        std::array<frc::SwerveModuleState, 4> moduleStates; /* FR, FL, RR, RL */

        /* Control states for the DT */

        ControlState driveState = kOPEN_LOOP_ROBOT_REL;
        APPCDiscriptor params;
        std::shared_ptr<PurePursuitController> PPC;
        frc::ChassisSpeeds currState;

        std::shared_future<std::shared_ptr<rospathmsgs::srv::GetPath_Response>> future;
        bool hasPathStart = true;
        std::string pathQueued = "";

        void execActions();

        void updateSensorData();

        //void checkDeltaCurrent(double, double, double, double);

        frc::ChassisSpeeds twistDrive(const geometry_msgs::msg::Twist &twist, const frc::Rotation2d &orientation0);
        frc::ChassisSpeeds twistDrive(const geometry_msgs::msg::Twist &twist);

        /* ROS Publishers */

        ROS_PUB(std_msgs::msg::Float32) drivetrainHeadingPub;
        std_msgs::msg::Float32 drivetrainHeadingMsg;

        ROS_PUB(geometry_msgs::msg::Pose2D) robotPositionPub;
        geometry_msgs::msg::Pose2D robotPositionMsg;
        frc::Pose2d pose;

        ROS_PUB(geometry_msgs::msg::Twist) robotVelocityPub;
        geometry_msgs::msg::Twist robotVelocityMsg;

        ROS_PUB(rospathmsgs::msg::Waypoint) autoLookaheadPointPub;
        rospathmsgs::msg::Waypoint lookAheadPoint;

        ROS_PUB(std_msgs::msg::Float32) autoToLookaheadAnglePub;
        std_msgs::msg::Float32 autoToLookaheadAngleMsg;

        ROS_PUB(std_msgs::msg::Int16) driveControlModePub;
        std_msgs::msg::Int16 driveControlModeMsg;

        ROS_PUB(std_msgs::msg::Int16) allianceColorPub;

        bool headingControlUpdate = false;

        /* Controller publishing (system independent only!) */
#ifdef SystemIndependent
        ROS_PUB(can_msgs::msg::MotorMsg) intakeDemandPublisher;
        can_msgs::msg::MotorMsg intakeDemandMsg;

        ROS_PUB(std_msgs::msg::Int16) intakeSoleDemandPublisher;
        std_msgs::msg::Int16 intakeSoleDemandMsg;

        ROS_PUB(can_msgs::msg::MotorMsg) indexerDemandPublisher;
        can_msgs::msg::MotorMsg indexerDemandMsg;

        ROS_PUB(can_msgs::msg::MotorMsg) deliveryDemandPublisher;
        can_msgs::msg::MotorMsg deliveryDemandMsg;

        ROS_PUB(can_msgs::msg::MotorMsg) flywheelDemandPublisher;
        can_msgs::msg::MotorMsg flywheelDemandMsg;

        ROS_PUB(can_msgs::msg::MotorMsg) hoodDemandPublisher;
        can_msgs::msg::MotorMsg hoodDemandMsg;

        ROS_PUB(can_msgs::msg::MotorMsg) climberLDemandPublisher;
        ROS_PUB(can_msgs::msg::MotorMsg) climberRDemandPublisher;
        can_msgs::msg::MotorMsg climberDemandMsg;
#endif

        /* ROS Subscibers */

        ROS_SUB(sensor_msgs::msg::Joy) stickSub0;
        void setStick0(const sensor_msgs::msg::Joy);
        geometry_msgs::msg::Twist stickTwist;
        sensor_msgs::msg::Joy lastStick0;


        ROS_SUB(sensor_msgs::msg::Joy) stickSub1;
        void setStick1(const sensor_msgs::msg::Joy);
        sensor_msgs::msg::Joy lastStick1;

        ROS_SUB(std_msgs::msg::Int16) DriveModeSub;
        void setDriveMode(const std_msgs::msg::Int16);

        ROS_SUB(std_msgs::msg::Int16) HeadingControlSub;
        ROS_SUB(std_msgs::msg::Float32) limelightRangeSub;
        void setLimelightRange(const std_msgs::msg::Float32);
        void setHeadingControlEnabled(const std_msgs::msg::Int16);
        void setHeadingControlSetpoint(double);
        void setAIAngleOffset(const std_msgs::msg::Float32);
        PIDF headingController = PIDF(HEADING_CONTROL_GAINS_TELE, "gyro_pid");
        int headingControl = 0; /* (0, disabled), (1, gyroLock), (2, limelightAngle) */
        double headingControlSetpoint = 0;
        double range = 0;

        ROS_SUB(std_msgs::msg::Float32) limelightAngleOffsetSub;
        ROS_SUB(std_msgs::msg::Float32) aiAngleOffsetSub;
        void setLimelightAngleOffset(const std_msgs::msg::Float32);

        #ifdef SystemIndependent
        ROS_SUB(std_msgs::msg::Bool) hoodResetSub;
        void setHoodReset(const std_msgs::msg::Bool);
        #endif

        /* ROS services */

        ROS_SERVICE(autobt_msgs::srv::StringService) startPath;
        void enablePathFollowerS(std::shared_ptr<autobt_msgs::srv::StringService_Request>, std::shared_ptr<autobt_msgs::srv::StringService_Response>);

        ROS_SERVICE(can_msgs::srv::SetPIDFGains) setGyroGains;
        void updateGyroPIDGains(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request>,
                                std::shared_ptr<can_msgs::srv::SetPIDFGains::Response>);
        ROS_SERVICE(can_msgs::srv::SetPIDFGains) setXGains;
        void updateXPIDGains(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request>,
                                std::shared_ptr<can_msgs::srv::SetPIDFGains::Response>);
        ROS_SERVICE(can_msgs::srv::SetPIDFGains) setYGains;
        void updateYPIDGains(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request>,
                                std::shared_ptr<can_msgs::srv::SetPIDFGains::Response>);

        /* Ros clients */

        ROS_CLIENT(rospathmsgs::srv::GetPath) GPClient;
        rospathmsgs::srv::GetPath::Request::SharedPtr GPReq;

        /* Last update time for safety critical topics */
        double lastStickTime;

        /* Button bool for robot relative drive */
        bool isRobotRel = false;

        /* A set of three buttons for a toggle function that set the robot to tank mode */

        bool tankLockState = false;
        bool tankLockHeld = false;
        bool tankLockButton = false;

        double targetAngleOffset = 0;
        double ballAngleOffset = 0;
        /* Button for gyro reset */
        bool gyroReset = false;
        /* Button for intake */

#ifdef SystemIndependent
        bool intake = false;
        bool intakeDeploy = false;
        bool intakeRetract = false;
        bool unintake = false;
        bool shoot = false;

        bool hoodReset = 0;
        double hoodDemand = 0;

        bool climberEnabled = false;
        double climberDemand = 0;

        bool flywheelState = false;
        bool flywheelHeld = false;
        bool flywheelButton = false;
#endif

        /* Vector of iterators to check if currents values are too high */
        std::vector<double> currentIterators = {0, 0, 0, 0};
    };

} // namespace robot
