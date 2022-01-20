#include "robot_lib/PurePursuitController.h"
#include "Constants.h"
#include <units/math.h>
#include <units/angle.h>
#include <units/velocity.h>
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

    double PurePursuitController::getDist(rospathmsgs::msg::Waypoint pos1, rospathmsgs::msg::Waypoint pos2) {
        double dx = pos2.point.x - pos1.point.x;
        double dy = pos2.point.y - pos1.point.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double PurePursuitController::getRemainingDistance(frc::Pose2d currPos){
        return mPath.size() / 100.0 + getDist(currPos, mPath.top());

    }

    bool PurePursuitController::isDone(frc::Pose2d pos) {
        if(mPath.size() == 1) {
            return getDist(pos, mPath.top()) <= mParams.mPathCompletionTolerance;
        }
        return false;
    }

    void PurePursuitController::setPath(std::stack<rospathmsgs::msg::Waypoint> path){
        mPath = path;
    }
    //requires the stack to have at least one value walks to local minima of the path as compared to the robot
    void PurePursuitController::walkToClosest(frc::Pose2d currPose) {
        rospathmsgs::msg::Waypoint nextPoint = mPath.top();
        double lastDistance = getDist(currPose, nextPoint);
        double currDistance = lastDistance;
        while(currDistance <= lastDistance && mPath.size() > 1) {
            mPath.pop();
            lastDistance = currDistance;
            currDistance = getDist(currPose, mPath.top());
        }
    }

    rospathmsgs::msg::Waypoint PurePursuitController::getLookAheadPoint(double dist) {
        //.01 spaceing between points
        double currDist = 0;
        std::stack<rospathmsgs::msg::Waypoint> storage;
        rospathmsgs::msg::Waypoint currPoint = mPath.top();
        rospathmsgs::msg::Waypoint lookAhead;
        // go through, and while the path still exsists, take the next point in the path and find the distance between it and the current point
        while(currDist < dist && mPath.size() > 1)
        {
            lookAhead = mPath.top();
            storage.push(lookAhead);
            mPath.pop();
            currDist += getDist(lookAhead, currPoint);
            currPoint = lookAhead;
        }
        lookAhead = mPath.top();
        while(storage.size() > 0)
        {
            mPath.push(storage.top());
            storage.pop();
        }
        return mPath.top();
    }

    Circle PurePursuitController::joinPath(frc::Pose2d currPos, rospathmsgs::msg::Waypoint lookAheadPoint){
        Circle arcDebug;
        double x1 = currPos.Translation().X().to<double>();
        double y1 = currPos.Translation().Y().to<double>();
        double x2 = lookAheadPoint.point.x;
        double y2 = lookAheadPoint.point.y;

        frc::Translation2d poseToLookahead = -(currPos.Translation()) + frc::Translation2d{units::meter_t{lookAheadPoint.point.x}, units::meter_t{lookAheadPoint.point.y}};
        double cross_product = poseToLookahead.X().to<double>() * currPos.Rotation().Sin()
                - poseToLookahead.Y().to<double>() * currPos.Rotation().Cos();
        if (std::abs(cross_product) < kEpsilon) {
            arcDebug.exsists = false;
            return arcDebug;
        }

        double dx = x1 - x2;
        double dy = y1 - y2;
        double my = (cross_product > 0 ? -1 : 1) * currPos.Rotation().Cos();
        double mx = (cross_product > 0 ? 1 : -1) * currPos.Rotation().Sin();

        double cross_term = mx * dx + my * dy;

        if (std::abs(cross_term) < kEpsilon) {
            arcDebug.exsists = false;
            return arcDebug;
        }
        arcDebug.center = frc::Translation2d{units::meter_t{(mx * (x1 * x1 - x2 * x2 - dy * dy) + 2 * my * x1 * dy) / (2 * cross_term)},
                        units::meter_t{(-my * (-y1 * y1 + y2 * y2 + dx * dx) + 2 * mx * y1 * dx) / (2 * cross_term)}};
        arcDebug.radius = .5 * std::abs((dx * dx + dy * dy) / cross_term);
        arcDebug.isRight = cross_product > 0;
        arcDebug.exsists = true;
        return arcDebug;
    }

    frc::ChassisSpeeds PurePursuitController::update(frc::Pose2d currPos, frc::ChassisSpeeds currState, double now) {
        walkToClosest(currPos);
        if (isDone(currPos)) {
            return frc::ChassisSpeeds{units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}};
        }
        double distanceFromPath = getDist(currPos, mPath.top());
        rospathmsgs::msg::Waypoint lookAheadPoint = getLookAheadPoint(distanceFromPath + mParams.mFixedLookahead);
        Circle circle = joinPath(currPos, lookAheadPoint); //this should throw stuff back to the stack to add a circle to the path if need be
        double speed = lookAheadPoint.velocity;

        // Ensure we don't accelerate too fast from the previous command
        double dt = now - mParams.mLastTime;
        double accel = (speed - mLastCommand.vx.to<double>()) / dt;
        if (accel < -mParams.mMaxAccel) {
            speed = mLastCommand.vx.to<double>() - mParams.mMaxAccel * dt;
        } else if (accel > mParams.mMaxAccel) {
            speed = mLastCommand.vx.to<double>() + mParams.mMaxAccel * dt;
        }

        // Ensure we slow down in time to stop
        // vf^2 = v^2 + 2*a*d
        // 0 = v^2 + 2*a*d
        double remaining_distance = getRemainingDistance(currPos);
        double max_allowed_speed = std::sqrt(2 * mParams.mMaxAccel * remaining_distance);
        if (std::abs(speed) > max_allowed_speed) {
            speed = max_allowed_speed * (speed / std::abs(speed));
        }

        frc::ChassisSpeeds rv;
        //.01 added to make the math happy?
        frc::Rotation2d inertialHeading = frc::Rotation2d(currState.vx.to<double>(), currState.vy.to<double>());
        if (circle.exsists) {
            inertialHeading.RotateBy(units::radian_t{(circle.isRight ? -1 : 1) * std::abs(speed) / circle.radius});
            rv = frc::ChassisSpeeds{units::meters_per_second_t{speed * inertialHeading.Cos()}, units::meters_per_second_t{speed * inertialHeading.Sin()}, units::radians_per_second_t{0}};
        } else {
            //need to implement the whole omega turning thing... kinda importaint
            rv = frc::ChassisSpeeds{units::meters_per_second_t{speed * inertialHeading.Cos()}, units::meters_per_second_t{speed * inertialHeading.Sin()}, units::radians_per_second_t{0}};
        }
        mParams.mLastTime = now;
        mLastCommand = rv;
        return rv;
        
    }
} // namespace robot

