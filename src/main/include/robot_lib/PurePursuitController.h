#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Twist2d.h>
#include <rospathmsgs/msg/waypoint.hpp>
#include <frc/geometry/Pose2d.h>
#include <rclcpp/rclcpp.hpp>
#include <stack>
#include <robot_lib/util/PIDF.h>

#define _USE_MATH_DEFINES
#include <cmath>

namespace robot
{
    const double kEpsilon = 1E-9;

    struct updateReturnType
    {
        frc::ChassisSpeeds speed;
        rospathmsgs::msg::Waypoint lookaheadPoint;
        frc::Rotation2d inertialHeading;
    };

    struct APPCDiscriptor
    {
        double mFixedLookahead;
        double mVelocityLookaheadCoeff;
        double mLastTime;
        double mMaxAccel;
        double mDt;
        double mPathCompletionTolerance;
        double mMaxRotVel;
        PIDFDiscriptor mXPIDFDescriptor;
        PIDFDiscriptor mYPIDFDescriptor;
    };

    struct Circle
    {
        frc::Rotation2d curvature;
        bool isRight;
        bool exsists;
    };

    class PurePursuitController
    {
    public:
        frc::Rotation2d inertialHeading;
        PurePursuitController(APPCDiscriptor params);
        void setPath(std::stack<rospathmsgs::msg::Waypoint> mPath);
        bool isDone(frc::Pose2d pos);
        void setXPIDF(PIDFDiscriptor);
        void setYPIDF(PIDFDiscriptor);
        void createRosBindings(rclcpp::Node*);
        updateReturnType update(frc::Pose2d currPos, frc::ChassisSpeeds currState, double now);
        rospathmsgs::msg::Waypoint mLastpoint;
        

    private:
        PIDF xPID = PIDF(PIDFDiscriptor{0, 0, 0, 0}, "xPID");
        PIDF yPID = PIDF(PIDFDiscriptor{0, 0, 0, 0}, "yPID");
        rclcpp::Publisher<rospathmsgs::msg::Waypoint>::SharedPtr lookaheadPub;

        APPCDiscriptor mParams;
        std::stack<rospathmsgs::msg::Waypoint> mPath;
        frc::ChassisSpeeds mLastCommand;
        static double getDist(frc::Pose2d pos1, rospathmsgs::msg::Waypoint pos2);
        static double getDist(rospathmsgs::msg::Waypoint pos1, rospathmsgs::msg::Waypoint pos2);
        void walkToClosest(frc::Pose2d currPos);
        frc::Rotation2d joinPath(frc::Pose2d currPos, rospathmsgs::msg::Waypoint lookAheadPoint);
        double getRemainingDistance(frc::Pose2d currPos);
        rospathmsgs::msg::Waypoint getLookAheadPoint(double lookAheadDist);

        
    };

} // namespace robot