#pragma once

#include "subsystems/Subsystem.h"
#include "robot_lib/PurePursuitController.h"
#include <ctre/Phoenix.h>
#include <frc/controller/RamseteController.h>
#include <rclcpp/rclcpp.hpp>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include "robot_lib/SModule.h"
#include "rospathmsgs/srv/get_path.hpp"
#include "Constants.h"
#include "robot_lib/PurePursuitController.h"
#include "robot_lib/util/PIDF.h"

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

namespace robot
{

    /**
     * Possible control states for the drivetrain to be in
     **/
    enum ControlState
    {
        OPEN_LOOP_ROBOT_REL,
        OPEN_LOOP_FIELD_REL,
        VELOCITY_TWIST,
        PURSUIT
    };

    class Drivetrain : public Subsystem
    {
    public:
        Drivetrain();

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

        /**
         * Request a path from the path generator with the given name, assuming it is baked
         */
        bool enablePathFollower(std::string name);

        void enableOpenLoop();

        void resetPose();

    private:
        bool DEBUG = true;

        // IO devices
        std::shared_ptr<SModule> frontRMod, frontLMod, rearRMod, rearLMod;
        std::vector<std::shared_ptr<SModule>> sModules;
        std::shared_ptr<PigeonIMU> imu;

        // underlying controllers
        frc::Translation2d sFrontRight{CHASSIS_LENGTH, CHASSIS_LENGTH};
        frc::Translation2d sFrontLeft{CHASSIS_LENGTH, -CHASSIS_LENGTH};
        frc::Translation2d sRearRight{-CHASSIS_LENGTH, CHASSIS_LENGTH};
        frc::Translation2d sRearLeft{-CHASSIS_LENGTH, -CHASSIS_LENGTH};
        frc::SwerveDriveKinematics<4> sKinematics{sFrontRight, sFrontLeft, sRearRight, sRearLeft};
        frc::SwerveDriveOdometry<4> sOdom{sKinematics, frc::Rotation2d{units::degree_t{0}}};
        std::array<frc::SwerveModuleState, 4> moduleStates; // fr, fl, rr, rl

        // Control states for the DT
        ControlState driveState = OPEN_LOOP_ROBOT_REL;
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

        // ROS Publishers

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr drivetrainHeadingPub;
        std_msgs::msg::Float32 drivetrainHeadingMsg;

        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr robotPositionPub;
        geometry_msgs::msg::Pose2D robotPositionMsg;
        frc::Pose2d pose;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robotVelocityPub;
        geometry_msgs::msg::Twist robotVelocityMsg;

        rclcpp::Publisher<rospathmsgs::msg::Waypoint>::SharedPtr autoLookaheadPointPub;
        rospathmsgs::msg::Waypoint lookAheadPoint;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr autoToLookaheadAnglePub;
        std_msgs::msg::Float32 autoToLookaheadAngleMsg;

        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr driveControlModePub;
        std_msgs::msg::Int16 driveControlModeMsg;

// Controller publishing (system independent only!)
#ifdef SystemIndependent
        rclcpp::Publisher<can_msgs::msg::MotorMsg>::SharedPtr intakeDemandPublisher;
        can_msgs::msg::MotorMsg intakeDemandMsg;

        rclcpp::Publisher<can_msgs::msg::MotorMsg>::SharedPtr indexerDemandPublisher;
        can_msgs::msg::MotorMsg indexerDemandMsg;

        rclcpp::Publisher<can_msgs::msg::MotorMsg>::SharedPtr deliveryDemandPublisher;
        can_msgs::msg::MotorMsg deliveryDemandMsg;

        rclcpp::Publisher<can_msgs::msg::MotorMsg>::SharedPtr flywheelDemandPublisher;
        can_msgs::msg::MotorMsg flywheelDemandMsg;

        rclcpp::Publisher<can_msgs::msg::MotorMsg>::SharedPtr hoodDemandPublisher;
        can_msgs::msg::MotorMsg hoodDemandMsg;
#endif

        // ROS Subscibers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr stickSub0;
        void setStick0(const sensor_msgs::msg::Joy);
        geometry_msgs::msg::Twist stickTwist;
        sensor_msgs::msg::Joy lastStick0;


        #ifdef SystemIndependent
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr stickSub1;
        void setStick1(const sensor_msgs::msg::Joy);
        sensor_msgs::msg::Joy lastStick1;
        #endif

        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr DriveModeSub;
        void setDriveMode(const std_msgs::msg::Int16);

        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr HeadingControlSub;
        void setHeadingControlEnabled(const std_msgs::msg::Int16);
        void setHeadingControlSetpoint(double newHeadingSetpoint);
        PIDF headingController = PIDF(PIDFDiscriptor{.018, .001, 0, 0}, "gyro_pid");
        int headingControl = 0; //(0, disabled), (1, gyroLock), (2, limelightAngle)
        double headingControlSetpoint = 0;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr limelightAngleOffsetSub;
        void setLimelightAngleOffset(const std_msgs::msg::Float32);

        #ifdef SystemIndependent
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hoodResetSub;
        void setHoodReset(const std_msgs::msg::Bool);
        #endif

        // Ros services
        rclcpp::Service<autobt_msgs::srv::StringService>::SharedPtr startPath;
        void enablePathFollowerS(std::shared_ptr<autobt_msgs::srv::StringService_Request>, std::shared_ptr<autobt_msgs::srv::StringService_Response>);

        rclcpp::Service<can_msgs::srv::SetPIDFGains>::SharedPtr setGyroGains;
        void updateGyroPIDGains(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request>,
                                std::shared_ptr<can_msgs::srv::SetPIDFGains::Response>);
        rclcpp::Service<can_msgs::srv::SetPIDFGains>::SharedPtr setXGains;
        void updateXPIDGains(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request>,
                                std::shared_ptr<can_msgs::srv::SetPIDFGains::Response>);
        rclcpp::Service<can_msgs::srv::SetPIDFGains>::SharedPtr setYGains;
        void updateYPIDGains(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request>,
                                std::shared_ptr<can_msgs::srv::SetPIDFGains::Response>);

        // Ros clients
        rclcpp::Client<rospathmsgs::srv::GetPath>::SharedPtr GPClient;
        rospathmsgs::srv::GetPath::Request::SharedPtr GPReq;

        // last update time for safety critical topics
        double lastStickTime;

        // button bool for robot relitive drive
        bool isRobotRel = false;
        // a set of three buttons for a toggle function that set the robot to tank mode
        bool tankLockState = false;
        bool tankLockHeld = false;
        bool tankLockButton = false;

        double angleOffset = 0;
        // button for gyro reset
        bool gyroReset = false;
        // button for intake

#ifdef SystemIndependent
        bool intake = false;
        bool unintake = false;
        bool shoot = false;

        bool hoodReset = 0;
        double hoodDemand = 0;

        bool flywheelState = false;
        bool flywheelHeld = false;
        bool flywheelButton = false;
#endif

        // vector of iterators to check if currents values are too high
        std::vector<double> currentIterators = {0, 0, 0, 0};
    };

} // namespace robot
