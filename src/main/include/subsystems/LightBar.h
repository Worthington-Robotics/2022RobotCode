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
     * Possible control states for the LightStrip to be in
     **/
    enum LightState
    {
        TARGETING, INVENTORY, ALLIANCE, RAINBOW, BATTERY, TEMPERATURE;
    };

    class LightBar : public Subsystem
    {
        public:

        LightBar();

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

        void lightModeCallback();

        /**
         * Callbacks for ROS Subscribers 
         **/

        // /**
        //  * Callback for streaming generated trajectories into the trajectory follower
        //  **/
        // void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    private:

        void execActions();

        //void updateSensorData();

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

        // Control states for the lights
        LightState lightState = BATTERY;
    };

} // namespace robot
