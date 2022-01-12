#pragma once

#include <ctre/Phoenix.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Twist2d.h>
#include <rospathmsgs/msg/waypoint.hpp>
#include <frc/geometry/Pose2d.h>
#include <stack>

#define _USE_MATH_DEFINES
#include <cmath>

namespace robot
{
    const double kEpsilon = 1E-9;
    struct APPCDiscriptor
    {
        double mFixedLookahead;
        std::stack<rospathmsgs::msg::Waypoint> mPath;
        frc::Twist2d mLastCommand;
        double mLastTime;
        double mMaxAccel;
        double mDt;
        double mPathCompletionTolerance;

    };

    class PurePursuitController
    {
    public:
        PurePursuitController(APPCDiscriptor params);
        bool isDone(frc::Pose2d pos);
        frc::Twist2d update(frc::Pose2d robot_pose, double now);
        void walkToClosest(frc::Pose2d currPos);


    private:
        APPCDiscriptor mParams;
        double getDist(frc::Pose2d pos1, rospathmsgs::msg::Waypoint pos2);

        
    };

} // namespace robot

