// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include "subsystems/drivetrain.h"
#include "subsystems/userinput.h"
#include "subsystems/LightBar.h"
#include "subsystems/Battery.h"
#include "Constants.h"
#include "robot_lib/VersionData.h"

void Robot::RobotInit()
{
    rclcpp::init(0, NULL);

    frc::ReportError(frc::warn::Warning, "Robot.cpp", 16, "RobotInit()", "ROS Sucessfully Init!");

    // construct subsystems
    auto drive = std::make_shared<robot::Drivetrain>();
    auto sticks = std::make_shared<robot::UserInput>();
    auto lights = std::make_shared<robot::LightBar>();
    auto battery = std::make_shared<robot::Battery>();
    sticks->registerSticks(USER_STICKS); //  register which joystick IDs to read

    // intialize all subsystems here
    manager = std::make_shared<robot::SubsystemManager>();
    manager->registerSubsystems(std::vector<std::shared_ptr<robot::Subsystem>>{
        drive,
        sticks,
        lights,
        battery});

    // grab the version string
    robot::ShowVersionData();
}

void Robot::RobotPeriodic()
{
    //std::cout << "spinning" << std::endl;
}

void Robot::AutonomousInit()
{
    manager->stopDisabledLoop();
    manager->startEnabledLoop();
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    manager->stopDisabledLoop();
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

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
