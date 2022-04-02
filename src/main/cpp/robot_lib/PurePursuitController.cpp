#include "robot_lib/PurePursuitController.h"
#include "Constants.h"
#include <units/math.h>
#include <units/angle.h>
#include <units/velocity.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

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

    PurePursuitController::PurePursuitController(APPCDiscriptor params)
    {
        mParams = params;
        setXPIDF(mParams.mXPIDFDescriptor);
        setYPIDF(mParams.mYPIDFDescriptor);
    }

    double PurePursuitController::getDist(frc::Pose2d pos1, rospathmsgs::msg::Waypoint pos2)
    {
        double dx = pos2.point.x - pos1.X().to<double>();
        double dy = pos2.point.y - pos1.Y().to<double>();
        return std::sqrt(dx * dx + dy * dy);
    }

    double PurePursuitController::getDist(rospathmsgs::msg::Waypoint pos1, rospathmsgs::msg::Waypoint pos2)
    {
        double dx = pos2.point.x - pos1.point.x;
        double dy = pos2.point.y - pos1.point.y;
        // sqrt(x^2+ y^2)
        return std::sqrt(dx * dx + dy * dy);
    }

    double PurePursuitController::getRemainingDistance(frc::Pose2d currPos)
    {
        return mPath.size() / 100.0 + getDist(currPos, mPath.top());
    }

    bool PurePursuitController::isDone(frc::Pose2d pos)
    {
        return getDist(pos, mLastpoint) <= mParams.mPathCompletionTolerance;
    }

    void PurePursuitController::setPath(std::stack<rospathmsgs::msg::Waypoint> path)
    {
        mPath = path;
        // std::cout << mPath.size() << " is the total number of points in the path STACK" << std::endl;
    }

    // requires the stack to have at least one value walks to local minima of the path as compared to the robot
    void PurePursuitController::walkToClosest(frc::Pose2d currPose)
    {
        rospathmsgs::msg::Waypoint currPoint = mPath.top();
        rospathmsgs::msg::Waypoint lastPoint = currPoint;
        double currDistance = getDist(currPose, currPoint);
        double lastDistance = currDistance;
        while (currDistance <= lastDistance && mPath.size() > 1)
        {
            lastPoint = currPoint;
            lastDistance = currDistance;
            mPath.pop();
            currPoint = mPath.top();
            currDistance = getDist(currPose, currPoint);
            // std::cout << lastDistance << " // " << currDistance << std::endl;
        }
        mPath.push(lastPoint);
    }

    rospathmsgs::msg::Waypoint PurePursuitController::getLookAheadPoint(double dist)
    {
        //.01 spaceing between points
        double currDist = 0;
        std::stack<rospathmsgs::msg::Waypoint> storage;
        rospathmsgs::msg::Waypoint currPoint = mPath.top();
        rospathmsgs::msg::Waypoint lookAhead;
        // go through, and while the path still exsists, take the next point in the path and find the distance between it and the current point
        while (currDist < dist && mPath.size() > 1)
        {
            lookAhead = mPath.top();
            storage.push(lookAhead);
            mPath.pop();
            currDist += getDist(lookAhead, currPoint);
            currPoint = lookAhead;
        }
        lookAhead = mPath.top();
        // std::cout << " amount of points looked ahead: " << storage.size();
        while (storage.size() > 0)
        {
            mPath.push(storage.top());
            storage.pop();
        }
        return lookAhead;
    }

    
    void PurePursuitController::setXPIDF(PIDFDiscriptor disc){
        xPID.setPIDFDisc(disc);
    }

    void PurePursuitController::setYPIDF(PIDFDiscriptor disc){
        yPID.setPIDFDisc(disc);
    }
    
    void PurePursuitController::createRosBindings(rclcpp::Node* node){
        xPID.createRosBindings(node);
        yPID.createRosBindings(node);
    }

    frc::Rotation2d PurePursuitController::joinPath(frc::Pose2d currPos, rospathmsgs::msg::Waypoint lookAheadPoint)
    {
        Circle arcDebug;
        double x1 = currPos.Translation().X().to<double>();
        double y1 = currPos.Translation().Y().to<double>();
        double x2 = lookAheadPoint.point.x;
        double y2 = lookAheadPoint.point.y;

        double deltaX = (x2 - x1);
        double deltaY = (y2 - y1);

        frc::Rotation2d angle = frc::Rotation2d(deltaX, deltaY);
        return angle;
    }

    updateReturnType PurePursuitController::update(frc::Pose2d currPos, frc::ChassisSpeeds currState, double now)
    {
        //Check if the path is done, if it is, send a stop command and nothing else
        if (isDone(currPos))
        {
            return {frc::ChassisSpeeds{units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}}};
        }
        //if we *aren't* done, walk to the closest point on the path 
        walkToClosest(currPos);
        //figure out how fast we're going so we know how far to look ahead
        double totalVel = std::sqrt(currState.vx.to<double>() * currState.vx.to<double>() + currState.vy.to<double>() * currState.vy.to<double>());
        //figure out *where* we're looking ahead too, that .02 is a constant that makes sure if we're stopped, we are still looking ahead on the path
        auto lookAheadPoint = getLookAheadPoint(mParams.mVelocityLookaheadCoeff * totalVel + mParams.mFixedLookahead);
        //get the angle to our lookahead point, this is where we're going
        frc::Rotation2d angleToNextPoint = joinPath(currPos, lookAheadPoint); 
        //figure out how fast we're supposed to be going, this is encoded within the path
        xPID.setSetpoint(lookAheadPoint.point.x, false);
        yPID.setSetpoint(lookAheadPoint.point.y, false);
        double xSpeed = xPID.update(currPos.Translation().X().to<double>());
        double ySpeed = yPID.update(currPos.Translation().Y().to<double>());
        //convert where we should be going to relitive to the robot, so we can actually get there
        //TRIG ;-; (used to turn the speed into a vector)
        frc::ChassisSpeeds rv = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t{.83 * lookAheadPoint.velocity * angleToNextPoint.Cos() - xSpeed}, units::meters_per_second_t{.83 * lookAheadPoint.velocity * angleToNextPoint.Sin() - ySpeed}, units::radians_per_second_t{0}, currPos.Rotation());
        //std::cout << totalVel << std::endl;
        return {rv, lookAheadPoint, inertialHeading};
    }
} // namespace robot
