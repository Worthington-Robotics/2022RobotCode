#pragma once

#include "subsystems/Subsystem.h"
#include "robot_lib/PurePursuitController.h"
#include "robot_lib/SModule.h"
#include "Constants.h"
#include "Util.h"
#include "robot_lib/PurePursuitController.h"
#include "robot_lib/util/PIDF.h"

#include <ctre/Phoenix.h>
#include <rospathmsgs/srv/get_path.hpp>
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
#include <autobt_msgs/srv/string_service.hpp>

#define DEBUG_enable

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

        ROS_PUB(MSG_FLOAT) drivetrainHeadingPub;
        MSG_FLOAT drivetrainHeadingMsg;

        ROS_PUB(geometry_msgs::msg::Pose2D) robotPositionPub;
        geometry_msgs::msg::Pose2D robotPositionMsg;
        frc::Pose2d pose;

        ROS_PUB(geometry_msgs::msg::Twist) robotVelocityPub;
        geometry_msgs::msg::Twist robotVelocityMsg;

        ROS_PUB(rospathmsgs::msg::Waypoint) autoLookaheadPointPub;
        rospathmsgs::msg::Waypoint lookAheadPoint;

        ROS_PUB(MSG_FLOAT) autoToLookaheadAnglePub;
        MSG_FLOAT autoToLookaheadAngleMsg;

        ROS_PUB(MSG_INT) driveControlModePub;
        MSG_INT driveControlModeMsg;

        ROS_PUB(MSG_INT) allianceColorPub;

        bool headingControlUpdate = false;

        /* ROS Subscibers */

        ROS_SUB(MSG_JOY) stickSub0;
        void setStick0(const MSG_JOY);
        geometry_msgs::msg::Twist stickTwist;
        MSG_JOY lastStick0;


        ROS_SUB(MSG_JOY) stickSub1;
        void setStick1(const MSG_JOY);
        MSG_JOY lastStick1;

        ROS_SUB(MSG_INT) DriveModeSub;
        void setDriveMode(const MSG_INT);

        ROS_SUB(MSG_INT) HeadingControlSub;
        ROS_SUB(MSG_FLOAT) limelightRangeSub;
        void setLimelightRange(const MSG_FLOAT);
        void setHeadingControlEnabled(const MSG_INT);
        void setHeadingControlSetpoint(double);
        void setAIAngleOffset(const MSG_FLOAT);
        PIDF headingController = PIDF(HEADING_CONTROL_GAINS_TELE, "gyro_pid");
        int headingControl = 0; /* (0, disabled), (1, gyroLock), (2, limelightAngle) */
        double headingControlSetpoint = 0;
        double range = 0;

        ROS_SUB(MSG_FLOAT) limelightAngleOffsetSub;
        ROS_SUB(MSG_FLOAT) aiAngleOffsetSub;
        void setLimelightAngleOffset(const MSG_FLOAT);

        /* ROS services */

        ROS_SERVICE(MSG_STRING) startPath;
        void enablePathFollowerS(std::shared_ptr<MSG_STRING_Request>, std::shared_ptr<MSG_STRING_Response>);

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

        /* Vector of iterators to check if currents values are too high */
        std::vector<double> currentIterators = {0, 0, 0, 0};
    };

} // namespace robot
