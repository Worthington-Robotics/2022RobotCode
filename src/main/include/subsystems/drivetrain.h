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

    /**
     * A data struct that contains the data for all 
     * four swerve modules 
     **/
    struct SwerveSensorData{
        sSensorData frontLeft;
        sSensorData frontRight;
        sSensorData rearLeft;
        sSensorData rearRight;
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

        void enablePathFollowerS(std::shared_ptr<autobt_msgs::srv::StringService_Request> ping, std::shared_ptr<autobt_msgs::srv::StringService_Response> pong);
        
        void enableOpenLoop();
        
        void resetPose();

        void engageHeadingControl(std_msgs::msg::Bool engaged);

        void setHeadingControl(std_msgs::msg::Float32 setpoint);


        /**
         * Callbacks for ROS Subscribers 
         **/

        /**
         * Callback for streaming generated trajectories into the trajectory follower
         **/
        void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

        /**
         * Callback for streaming the current desired velocity twist of the drivetrain
         **/
        void twistCallback(const geometry_msgs::msg::Twist msg);

        /**
         * Callback for streaming joystick input into the drivetrain
         **/ 
        void stickCallback(const sensor_msgs::msg::Joy msg);

        /**
         * Callback for setting drivetrain modes. Selects between the control modes
         * enumerated in the ControlState enum.
         **/
        void driveModeCallback(const std_msgs::msg::Int16 msg);

    private:

        void execActions();

        void updateSensorData();

        void checkDeltaCurrent(double, double, double, double);

        frc::ChassisSpeeds twistDrive(const geometry_msgs::msg::Twist & twist, const frc::Rotation2d & orientation );
        frc::ChassisSpeeds twistDrive(const geometry_msgs::msg::Twist & twist );

        //IO devices
        std::shared_ptr<SModule> frontRMod, frontLMod, rearRMod, rearLMod;
        std::shared_ptr<PigeonIMU> imu;

        // ROS Publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yawPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr goalPub;
        #ifdef DEBUG_enable
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr currentAnglePub;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desiredAnglePub;
        #endif
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr autoTwistDemandPub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robotVelPub;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr robotPosPub;
        rclcpp::Publisher<rospathmsgs::msg::Waypoint>::SharedPtr lookaheadPointPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr inertialAnglePub;

        //Ros services
        rclcpp::Service<autobt_msgs::srv::StringService>::SharedPtr startPath;

        // ROS Messages for publishing
        std_msgs::msg::Float32 goal;
        #ifdef DEBUG_enable
            std_msgs::msg::Float32 currentAngle;
            std_msgs::msg::Float32 desiredAngle;
        #endif
        geometry_msgs::msg::Twist autoTwistDemand;
        std_msgs::msg::Float32 yaw;
        sensor_msgs::msg::Imu imuMsg;
        geometry_msgs::msg::Pose2D robotPosMsg;
        geometry_msgs::msg::Twist robotVelMsg;
        rospathmsgs::msg::Waypoint lookAheadPoint;
        std_msgs::msg::Float32 inertialAngle;


        // ROS Subscibers
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectorySub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr stickSub;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr DriveModeSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr HeadingSetpointSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr HeadingControlSub;

        // ROS SRV REQUESTS
        rclcpp::Client<rospathmsgs::srv::GetPath>::SharedPtr GPClient;
        rospathmsgs::srv::GetPath::Request::SharedPtr GPReq;

        // ROS Messages for storing subscription data
        geometry_msgs::msg::Twist stickTwist;
        geometry_msgs::msg::Twist lastTwist;
        sensor_msgs::msg::Joy lastStick;

        // underlying controllers
        frc::Translation2d sFrontRight{CHASSIS_LENGTH, CHASSIS_LENGTH};
        frc::Translation2d sFrontLeft{CHASSIS_LENGTH, -CHASSIS_LENGTH};
        frc::Translation2d sRearRight{-CHASSIS_LENGTH, CHASSIS_LENGTH};
        frc::Translation2d sRearLeft{-CHASSIS_LENGTH, -CHASSIS_LENGTH};
        frc::SwerveDriveKinematics<4> sKinematics {sFrontRight, sFrontLeft, sRearRight, sRearLeft};
        frc::SwerveDriveOdometry<4> sOdom {sKinematics, frc::Rotation2d{units::degree_t{0}}};
        std::array<frc::SwerveModuleState, 4> moduleStates; //fr, fl, rr, rl
        //std::shared_ptr<PurePursuitController> PPC;
        

        bool DEBUG = true;

        bool resetPIDVals = false;

        double kF;
        double kP;
        double kI;
        double kD;

        // Control states for the DT
        ControlState driveState = OPEN_LOOP_ROBOT_REL;
        SwerveSensorData moduleData;
        APPCDiscriptor params;
        std::shared_ptr<PurePursuitController> PPC;

        // last update time for safety critical topics
        double lastTwistTime, lastStickTime;

        // Demand variables
        double leftDemand, rightDemand;

        //
        bool headingControl = false;
        PIDF headingController = PIDF(PIDFDiscriptor{.033, 0, .0025, 0});

        //button bool for robot relitive drive
        bool isRobotRel = false;
        //a set of three buttons for a toggle function that set the robot to tank mode
        bool tankLockState = false;
        bool tankLockHeld = false;
        bool tankLockButton = false;
        //button bool for spin lock (NO GYRO PID, MAY DRIFT, FIX?)
        bool spinLock = false;
        //button for gyro reset
        bool gyroReset = false;

        frc::Pose2d pose;

        //vector of iterators to check if currents values are too high
        std::vector<double> iterators = {0, 0, 0, 0};
    };

} // namespace robot
