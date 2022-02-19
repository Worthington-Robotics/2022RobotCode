#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Twist2d.h>
#include <rospathmsgs/msg/waypoint.hpp>
#include <frc/geometry/Pose2d.h>
#include <rclcpp/rclcpp.hpp>
#include <stack>

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
        double mLastTime;
        double mMaxAccel;
        double mDt;
        double mPathCompletionTolerance;
        double mMaxRotVel;

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
        updateReturnType update(frc::Pose2d currPos, frc::ChassisSpeeds currState, double now);
        rospathmsgs::msg::Waypoint mLastpoint;
        

    private:
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