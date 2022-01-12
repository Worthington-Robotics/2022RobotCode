#include "robot_lib/PurePursuitController.h"
#include "Constants.h"
#include <units/math.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/geometry/Rotation2d.h>
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>


namespace robot
{
/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4
 * .pdf
 * 
 * Basically, we find a spot on the path we'd like to follow and calculate the
 * wheel speeds necessary to make us land on that spot. The target spot is a
 * specified distance ahead of us, and we look further ahead the greater our
 * tracking error.
 */

    PurePursuitController::PurePursuitController(APPCDiscriptor params) {
        mParams = params;
    }

    double PurePursuitController::getDist(frc::Pose2d pos1, rospathmsgs::msg::Waypoint pos2) {
        double dx = pos2.point.x - pos1.X().to<double>();
        double dy = pos2.point.y - pos1.Y().to<double>();
        return std::sqrt(dx * dx + dy * dy);
    }

    bool PurePursuitController::isDone(frc::Pose2d pos) {
        if(mParams.mPath.size() == 1) {
            return getDist(pos, mParams.mPath.top()) <= mParams.mPathCompletionTolerance;
        }
        return false;
    }
    //requires the stack to have at least one value walks to local minima of the path as compared to the robot
    void PurePursuitController::walkToClosest(frc::Pose2d currPose) {
        rospathmsgs::msg::Waypoint nextPoint = mParams.mPath.top();
        double lastDistance = getDist(currPose, nextPoint);
        double currDistance = lastDistance;
        while(currDistance <= lastDistance && mParams.mPath.size() > 1) {
            mParams.mPath.pop();
            lastDistance = currDistance;
            currDistance = getDist(currPose, mParams.mPath.top());
        }
    }

    frc::Twist2d PurePursuitController::update(frc::Pose2d robot_pose, double now) {
        /*frc::Pose2d pose = robot_pose;
        walkToClosest(pose);
        if (isDone(pose)) {
            return frc::Twist2d{units::meter_t{0}, units::meter_t{0}, units::degree_t{0}};
        }
        double distanceFromPath = getDist(pose, mParams.mPath.top());
        rospathmsgs::msg::Waypoint lookAheadPoint = getLookAheadPoint(distanceFromPath + mParams.mFixedLookahead);
        Optional<Circle> circle = joinPath(pose, lookahead_point.translation); //this should throw stuff back to the stack to add a circle to the path if need be
        double speed = lookAheadPoint.max_vel;
        // Ensure we don't accelerate too fast from the previous command
        double dt = now - mParams.mLastTime;
        if (mParams.mLastCommand == NULL) {
            mLastCommand = new RigidTransform2d.Delta(0, 0, 0);
            dt = mDt;
        }
        double accel = (speed - mLastCommand.dx) / dt;
        if (accel < -mMaxAccel) {
            speed = mLastCommand.dx - mMaxAccel * dt;
        } else if (accel > mMaxAccel) {
            speed = mLastCommand.dx + mMaxAccel * dt;
        }

        // Ensure we slow down in time to stop
        // vf^2 = v^2 + 2*a*d
        // 0 = v^2 + 2*a*d
        double remaining_distance = mPath.getRemainingLength();
        double max_allowed_speed = Math.sqrt(2 * mMaxAccel * remaining_distance);
        if (Math.abs(speed) > max_allowed_speed) {
            speed = max_allowed_speed * Math.signum(speed);
        }
        final double kMinSpeed = 4.0;
        if (Math.abs(speed) < kMinSpeed) {
            // Hack for dealing with problems tracking very low speeds with
            // Talons
            speed = kMinSpeed * Math.signum(speed);
        }

        RigidTransform2d.Delta rv;
        if (circle.isPresent()) {
            rv = new RigidTransform2d.Delta(speed, 0,
                    (circle.get().turn_right ? -1 : 1) * Math.abs(speed) / circle.get().radius);
        } else {
            rv = new RigidTransform2d.Delta(speed, 0, 0);
        }
        mLastTime = now;
        mLastCommand = rv;
        return rv;*/
        
    }
} // namespace robot

