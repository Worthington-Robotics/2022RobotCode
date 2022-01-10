#pragma once

#include "subsystems/Subsystem.h"
#include <ctre/Phoenix.h>
#include <frc/controller/RamseteController.h>
#include <rclcpp/rclcpp.hpp>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include "robot_lib/SModule.h"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16.hpp>

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
        void onLoop() override;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void publishData() override;

        void enableDebug(bool debug) override;

        /**
         * Callbacks for ROS Subscribers 
         **/

        /**
         * Callback for streaming generated trajectories into the trajectory follower
         **/
        void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

        /**
         * Calbback for streaming the current desired velocity twist of the drivetrain
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

        frc::ChassisSpeeds updateTrajectory(trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr nPose);

        frc::ChassisSpeeds twistDrive(const geometry_msgs::msg::Twist & twist, const frc::Rotation2d & orientation );
        frc::ChassisSpeeds twistDrive(const geometry_msgs::msg::Twist & twist );

        //IO devices
        std::shared_ptr<SModule> frontRMod, frontLMod, rearRMod, rearLMod;
        std::shared_ptr<PigeonIMU> imu;

        // ROS Publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr yawPub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheelStatePub;

        // ROS Messages for publishing
        std_msgs::msg::Int16 yaw;
        sensor_msgs::msg::Imu imuMsg;
        sensor_msgs::msg::JointState wheelState;

        // ROS Subscibers
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectorySub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr stickSub;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr DriveModeSub;

        // ROS Messages for storing subscription data
        geometry_msgs::msg::Twist stickTwist;
        geometry_msgs::msg::Twist lastTwist;
        sensor_msgs::msg::Joy lastStick;

        // underlying controllers
        frc::Translation2d sFrontRight{0.65_m, 0.65_m};
        frc::Translation2d sFrontLeft{0.65_m, -0.65_m};
        frc::Translation2d sRearRight{-0.65_m, 0.65_m};
        frc::Translation2d sRearLeft{-0.65_m, -0.65_m};
        frc::SwerveDriveKinematics<4> sKinematics {sFrontRight, sFrontLeft, sRearRight, sRearLeft};
        frc::SwerveDriveOdometry<4> sOdom {sKinematics, frc::Rotation2d{units::degree_t{0}}};
        std::array<frc::SwerveModuleState, 4> moduleStates; //fr, fl, rr, rl

        bool DEBUG = false;

        // Control states for the DT
        ControlState driveState = OPEN_LOOP_ROBOT_REL;

        // last update time for safety critical topics
        double lastTwistTime, lastStickTime;

        // Demand variables
        double leftDemand, rightDemand;

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
    };

} // namespace robot