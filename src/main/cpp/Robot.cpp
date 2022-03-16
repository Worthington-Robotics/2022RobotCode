// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include "Constants.h"
#include "robot_lib/VersionData.h"
#include "SubsystemManager.h"
#include "robot_lib/util/PIDF.h"

void Robot::RobotInit()
{
    std::cout << "code init started" << std::endl;
    
    const char* argv[] = {"--ros-args", "--log-level", "DEBUG"};
    rclcpp::init(3, argv);

    frc::ReportError(frc::warn::Warning, "Robot.cpp", 14, "RobotInit()", "ROS Sucessfully Init!");

    // construct subsystems
    externIO = std::make_shared<robot::ExternIO>();
    drive = std::make_shared<robot::Drivetrain>();
    sticks = std::make_shared<robot::UserInput>();
    lightBar = std::make_shared<robot::LightBar>();
  
    sticks->registerSticks(USER_STICKS); //  register which joystick IDs to read

    // intialize all subsystems here
    manager = std::make_shared<robot::SubsystemManager>();
    manager->registerSubsystems(std::vector<std::shared_ptr<robot::Subsystem>>{
         drive,
         sticks,
         externIO,
         lightBar
     });

    autoSel = std::make_shared<AutoSelect>(std::vector<std::string>({""}));
    autoSel->createRosBindings(manager.get());

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
    drive->resetPose();
    drive->reset();
    drive->setHeadingControlGains(HEADING_CONTROL_GAINS_AUTO);
    manager->startEnabledLoop();
    std::string autoSelect = frc::SmartDashboard::GetString("Auto Selector", "");
    std::cout << "auto selected: " << autoSelect << std::endl;
    autoSel->selectAuto(autoSelect);
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    manager->stopDisabledLoop();
    drive->reset();
    drive->enableOpenLoop();
    drive->setHeadingControlGains(HEADING_CONTROL_GAINS_TELE);
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
