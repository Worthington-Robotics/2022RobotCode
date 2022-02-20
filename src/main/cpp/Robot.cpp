// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include "Constants.h"
#include "robot_lib/VersionData.h"
#include "SubsystemManager.h"

void Robot::RobotInit()
{
    rclcpp::init(0, NULL);

    frc::ReportError(frc::warn::Warning, "Robot.cpp", 14, "RobotInit()", "ROS Sucessfully Init!");

    // construct subsystems
    drive = std::make_shared<robot::Drivetrain>();
    sticks = std::make_shared<robot::UserInput>();
    lights = std::make_shared<robot::LightBar>();
    
    sticks->registerSticks(USER_STICKS); //  register which joystick IDs to read

    // intialize all subsystems here
    manager = std::make_shared<robot::SubsystemManager>();
    manager->registerSubsystems(std::vector<std::shared_ptr<robot::Subsystem>>{
        drive,
        sticks,
        lights
    });

    // grab the version string
    robot::ShowVersionData();
}

void Robot::RobotPeriodic(){}

void Robot::AutonomousInit()
{
    manager->stopDisabledLoop();
    drive->resetPose();
    manager->startEnabledLoop();
    drive->enablePathFollower("six");
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    manager->stopDisabledLoop();
    drive->enableOpenLoop();
    manager->startEnabledLoop();
}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit()
{
    manager->stopEnabledLoop();
    manager->startDisabledLoop();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit()
{
    manager->stopDisabledLoop();
    manager->startEnabledLoop();
}
void Robot::TestPeriodic() {}

// std::shared_ptr<robot::SubsystemManager> Robot::getSubManager(){
//     if(!manager)
//     {
//         frc::ReportError(frc::err::NullParameter, "Robot.cpp", 70, "getSubManager", "Something has gone wrong with the creation of the subsystem manager" 
//         "if you are getting this error, then somehow you have requested the exsistance of the subsystem manager before the robot has fully inited. How you "
//         "did this, no one knows, but it deals with the non-singleton nature of the system :c");
//     }
//     return manager;
// }


#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
